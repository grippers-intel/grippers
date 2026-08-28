"""탑뷰 카메라 2대 + ArUco + geti 로 픽업 -> 이동 -> 내려놓기 미션을 라이브로 돌린다.

Host PC 가 하는 일은 딱 여기까지다: 매 사이클 로봇 pose(ArUco)와 기물 지도
(geti)를 계산해서 "지금 뭘 해야 하는지"(mode)와 "다음 좌표"를 VehicleLink 로
넘기는 것. 실제로 차를 움직이고 집고 내려놓는 건 차량(ROS2, Pi+Hailo)이
SmolVLA(그리퍼캠+차량 RGB캠)로 알아서 한다.

★ 차량에는 라이다가 있고, 여기서 모르는 장애물이 갑자기 나타나면 멈춰서 회피
기동을 하는 반사 안전 레이어가 따로 있다(차량 쪽 ROS2 노드 — 이 저장소 범위
밖). 그 레이어는 Host PC 와 무관하게 항상 최우선으로 작동해야 한다: 라이다는
차량에만 있고, Host PC 를 거치면 지연이 생겨 안전 기능으로 못 쓴다. 그래서 이
스크립트는 그 존재를 몰라도 안전하다 — 매 사이클 "지금 아는 최선의 좌표"만
계속 보내고, 차량이 회피 중이면 그 좌표를 무시하다가 끝나면 최신 좌표를
다시 따라가면 된다.

--vehicle-ip 를 안 주면 ConsoleVehicleLink 로 콘솔에 찍기만 한다(차량 없이
시험용). 주면 UdpVehicleLink 로 실제 UDP 전송한다 — 규격은
VEHICLE_LINK_PROTOCOL.md 참고.

라벨을 지정하지 않는다 — 화면에 보이는 기물 중 "지금 로봇 위치에서 가장
가까운 것"을 매번 골라서, 그 라벨에 맞는 상자(mission_config.PIECE_DEST_BOX:
체스말은 chess 상자, 나머지는 toy 상자)로 나른다. 하나 끝나면 멈추지 않고
다음 기물을 또 찾는다 — 화면(작업 영역)에 기물이 하나도 안 남을 때까지 반복.

사용법
    python run_mission.py
    python run_mission.py --cams 0 2
    python run_mission.py --show-cams   # 카메라 원본 창도 같이
    python run_mission.py --no-view
    python run_mission.py --mock-complete   # 차량 없이 전체 흐름만 시험
    python run_mission.py --step --mock-complete   # 단계마다 LiveMap 의 Next 버튼으로 직접 진행
    python run_mission.py --vehicle-ip 192.168.0.42   # 실제 차량(Pi)로 UDP 전송

화면은 기본으로 live_map.py 의 2D 지도(로봇/기물/상자/이동경로를 도형으로)
하나만 뜬다. 카메라 원본 + ArUco/geti 오버레이 창은 디버깅용이라 필요할 때만
--show-cams 로 따로 켠다.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import cv2

# config.py/localizer.py 는 aruco/ 하위폴더에 있다(팀원이 동기화하는 파일이라
# 건드리지 않는다) — 그 폴더를 경로에 추가해서 기존처럼 bare import 로 쓴다.
sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
from localizer import Camera, RobotLocalizer, detect, make_detector

import geti_detector
import piece_map
import window_layout
from live_map import LiveMap
from mission import MissionFSM, State
from run_localize import draw, open_cams
from vehicle_link import ConsoleVehicleLink, MissionCommand, UdpVehicleLink

_stop = False


def _on_sigint(signum, frame):
    global _stop
    _stop = True



def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--no-view", action="store_true")
    ap.add_argument("--seconds", type=float, default=None,
                    help="이 시간이 지나면 정지하고 종료한다. 예행연습 안전장치")
    ap.add_argument("--no-stop-on-enter", action="store_true",
                    help="Enter 로 멈추는 감시를 끈다(기본은 켜짐)")
    ap.add_argument("--display", type=int, default=0,
                    help="창을 띄울 화면. 0=주 화면, 1=오른쪽 확장 화면")
    ap.add_argument("--no-tile", action="store_true",
                    help="창 자동 배치를 끄고 OS 기본 위치에 맡긴다")
    ap.add_argument("--cam-width", type=int, default=None,
                    help="카메라 창 가로 크기(px). 안 주면 화면에 맞춰 자동으로 정한다")
    ap.add_argument("--show-cams", action="store_true",
                     help="카메라 원본 + ArUco/geti 오버레이 창도 같이 띄운다 (디버깅용)")
    ap.add_argument("--geti-device", type=str, default="CPU")
    ap.add_argument("--mock-complete", action="store_true",
                     help="차량이 아직 없을 때 GRASP/PLACE 를 즉시 완료된 것으로 흉내낸다(시험용)")
    ap.add_argument("--step", action="store_true",
                     help="단계마다 자동으로 안 넘어가고 LiveMap 의 Next 버튼을 눌러야 진행 "
                          "(조건 충족 여부는 버튼 옆 표시등 초록/빨강으로 보여줌)")
    ap.add_argument("--vehicle-ip", type=str, default=None,
                     help="차량(Pi) IP — 주면 실제 UDP로 전송(UdpVehicleLink), "
                          "안 주면 콘솔에만 찍는다(ConsoleVehicleLink)")
    ap.add_argument("--vehicle-cmd-port", type=int, default=5005)
    ap.add_argument("--vehicle-status-port", type=int, default=5006)
    ap.add_argument("--hz-every", type=int, default=20,
                    help="N 사이클마다 루프 Hz 와 단계별 소요를 출력한다(0이면 끄기)")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, _on_sigint)

    detector = make_detector()
    cams = [Camera.load(f"cam{i}", i) for i in args.cams]
    # ⚠️ Camera.load() 는 npz 가 없어도 예외를 내지 않고 HFOV 근사 행렬로
    # 조용히 넘어간다(config.py 주석 참고). run_localize.py 는 이 경고를
    # 찍는데 여기에는 없었다 — 미션을 실제로 돌리는 쪽이라 더 중요한데도
    # 근사값으로 도는지 사람이 알 방법이 없었다.
    #
    # 근사값과 실측의 차이는 작지 않다. C920 실측 fx=938.0 인데
    # HFOV 70.4° 근사는 fx=907.3 이다 — 약 3.4% 로, 1.35 m 거리에서
    # 4~5 cm 의 위치 오차가 된다.
    for c in cams:
        if not c.calibrated:
            print(f"⚠️ {c.name}: calib/cam*.npz 가 없어 HFOV {cfg.HFOV_DEG}° "
                  f"근사값을 씁니다 — 위치가 몇 cm 틀립니다. "
                  f"calibrate_camera.py 를 먼저 돌리세요.")
    caps = open_cams(args.cams)
    if not any(c.isOpened() for c in caps):
        print("\n열린 카메라가 하나도 없습니다. --cams 로 인덱스를 바꿔 보세요.")
        for c in caps:
            c.release()
        return 1

    print(f"geti 모델 불러오는 중 ({args.geti_device}, 카메라당 1개)...")
    # 카메라마다 별도 Deployment 인스턴스를 준다 — 하나를 공유하면 두 배경
    # 스레드가 동시에 infer() 를 불러서 "Infer Request is busy" 오류가 난다.
    workers = [geti_detector.GetiWorker(
        geti_detector.load_deployment(device=args.geti_device), c.name) for c in cams]
    print("geti 모델 준비 완료.")

    # 창 배치 계획. 실제로 옮기는 것은 첫 프레임을 그린 **뒤**다 —
    # OpenCV 는 창에 처음 그림을 넣을 때 창 크기를 그림 크기로 되돌린다.
    layout = None
    if not args.no_tile:
        layout = window_layout.plan(
            args.display, len(cams) if args.show_cams else 0,
            want_map=not args.no_view,
            cam_aspect=cfg.IMG_W / cfg.IMG_H,
            cam_width=args.cam_width)
        if layout is None:
            print("[display] 화면 정보를 못 읽어 자동 배치를 건너뜁니다 "
                  "(pyobjc 미설치?)")
    if args.show_cams:
        for cam in cams:
            cv2.namedWindow(cam.name, cv2.WINDOW_NORMAL)

    loc = RobotLocalizer()
    tracker = piece_map.PieceTracker()
    fsm = MissionFSM(manual_mode=args.step)
    if args.vehicle_ip:
        link = UdpVehicleLink(args.vehicle_ip, cmd_port=args.vehicle_cmd_port,
                              status_port=args.vehicle_status_port)
        print(f"차량 연결: UDP -> {args.vehicle_ip}:{args.vehicle_cmd_port} "
              f"(상태 수신: :{args.vehicle_status_port})")
    else:
        link = ConsoleVehicleLink(auto_complete=args.mock_complete)

    def _reset_all() -> None:
        # LiveMap 리셋 버튼 콜백 — 화면뿐 아니라 기물 추적/미션 상태도 같이 지운다.
        tracker.reset()
        fsm.reset()
        print("\n[live_map] 리셋됨 — 기물 추적/미션 상태 초기화\n")

    def _toggle_mode() -> None:
        # LiveMap Mode 버튼 콜백 — 자동↔수동 전환, 처음부터 다시 시작.
        fsm.set_manual_mode(not fsm.manual_mode)
        tracker.reset()
        print(f"\n[live_map] 모드 전환 -> {'MANUAL' if fsm.manual_mode else 'AUTO'} (초기화됨)\n")

    live_map = (LiveMap(on_reset=_reset_all, on_next=fsm.request_advance,
                        on_back=fsm.request_back, on_toggle_mode=_toggle_mode)
                if not args.no_view else None)

    print("\n시작 — 보이는 기물을 가까운 순서대로 라벨별 상자로 나릅니다"
          " (체스말→chess, 나머지→toy).")
    print("q 또는 Ctrl+C 로 종료\n")

    # --- Enter 로 즉시 정지 (2026-08-28, 실차 예행연습용) ---
    #
    # 실제로 바퀴가 도는 동안 사람이 손닿는 곳에 정지 수단이 있어야 한다.
    # cv2 창의 q 는 창에 포커스가 있어야 먹으므로 터미널에서는 못 쓴다.
    # ⚠️ stdin 이 TTY 가 아니면(백그라운드·nohup·파이프) readline 이 EOF 로
    # 즉시 돌아온다. 그걸 Enter 로 오해하면 기동하자마자 멈춘다 — 실제로
    # 그랬다. 사람이 칠 수 있는 터미널일 때만 감시를 건다.
    _enter_armed = (not args.no_stop_on_enter) and sys.stdin.isatty()
    if _enter_armed:
        def _enter_watch():
            global _stop
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if line == "":
                return          # EOF 는 Enter 가 아니다
            _stop = True
            print("\n[STOP] Enter — 정지합니다", flush=True)
        threading.Thread(target=_enter_watch, daemon=True).start()
        print("\n>>> Enter 를 치면 즉시 정지하고 종료합니다 <<<\n", flush=True)
    elif not args.no_stop_on_enter:
        print("\n[주의] stdin 이 터미널이 아니라 Enter 정지를 못 겁니다 — "
              "--seconds 로 시간 제한을 두세요\n", flush=True)

    _deadline = (time.time() + args.seconds) if args.seconds else None

    frames_seen = 0
    _placed = False
    # --- 루프 Hz 측정 (2026-08-28 HANDOFF §0-2) ---
    hz_n = 0
    hz_t0 = time.perf_counter()
    hz_acc = {"cap": 0.0, "geti": 0.0, "fsm": 0.0, "view": 0.0}
    try:
        # 라벨을 다 옮겨도 안 끝난다 — 새 기물이 놓이면 계속 반복
        while not _stop:
            if _deadline is not None and time.time() >= _deadline:
                print("\n[STOP] 제한 시간 — 정지합니다", flush=True)
                break
            _t = time.perf_counter()
            grabbed, dets = [], []
            for cap in caps:
                ok, frame = cap.read()
                grabbed.append(frame if ok else None)
                dets.append({} if not ok else
                            detect(detector, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)))

            pose = loc.update(cams, dets)
            _t_cap = time.perf_counter(); hz_acc["cap"] += _t_cap - _t

            preds = []
            for frame, worker in zip(grabbed, workers):
                if frame is None:
                    preds.append(None)
                    continue
                worker.submit(frame.copy())
                preds.append(worker.latest())

            obs_lists = [piece_map.pieces_from_prediction(cam, pred)
                         for cam, pred in zip(cams, preds)]
            pmap = tracker.update(obs_lists)
            _t_geti = time.perf_counter(); hz_acc["geti"] += _t_geti - _t_cap

            fsm.step(pose, pmap, link)
            _t_fsm = time.perf_counter(); hz_acc["fsm"] += _t_fsm - _t_geti
            frames_seen += 1

            if fsm.state == State.SEARCH_TARGET and frames_seen % 10 == 0:
                print(f"\r[SEARCH_TARGET] 작업 영역에 남은 기물 없음 — {pose}   ",
                      end="", flush=True)

            if live_map is not None:
                live_map.update(pose, pmap, goal=fsm.nav_goal, nav=fsm.last_nav,
                                 corner=fsm.nav_corner, path=fsm.nav_path,
                                 state_name=fsm.state.name, target_label=fsm.target_label,
                                 ready=(fsm.ready_to_advance if pose.ok else None),
                                 manual_mode=fsm.manual_mode, cmd=fsm.last_cmd)
                if live_map.closed():
                    break
            hz_acc["view"] += time.perf_counter() - _t_fsm

            hz_n += 1
            if args.hz_every and hz_n >= args.hz_every:
                _el = time.perf_counter() - hz_t0
                _ms = {k: v / hz_n * 1000 for k, v in hz_acc.items()}
                print(f"\n[hz] {hz_n / _el:.2f} Hz  ({_el / hz_n * 1000:.0f} ms/사이클)"
                      f"  캡처+ArUco {_ms['cap']:.0f}  geti {_ms['geti']:.0f}"
                      f"  FSM {_ms['fsm']:.0f}  화면 {_ms['view']:.0f} ms", flush=True)
                hz_n = 0
                hz_t0 = time.perf_counter()
                hz_acc = {k: 0.0 for k in hz_acc}

            if args.show_cams:
                for cam, frame, det, pred in zip(cams, grabbed, dets, preds):
                    if frame is None:
                        continue
                    disp = geti_detector.draw(frame, pred) if pred is not None else frame
                    cv2.imshow(cam.name, draw(disp, cam, det, pose))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

            if layout is not None and not _placed:
                # 창이 다 뜬 지금 한 번만 옮긴다. 매 사이클 옮기면 사람이
                # 손으로 옮겨 놓은 창을 계속 되돌려 버린다.
                print(window_layout.apply(
                    layout, [c.name for c in cams] if args.show_cams else [],
                    live_map.fig if live_map is not None else None), flush=True)
                _placed = True
    finally:
        # ⚠️ 링크를 그냥 닫으면 Pi 워치독(3사이클 = 0.3초)이 설 때까지
        # 바퀴가 돈다. 명시적으로 정지를 여러 번 보내 즉시 세운다.
        # UDP 라 한 발이 유실될 수 있으므로 연발한다.
        try:
            for _ in range(8):
                link.send(MissionCommand("stop", "SEARCH_TARGET", 0.0, 0.0, 0.0))
                time.sleep(0.05)
            print("[STOP] 정지 명령 8회 송신 완료")
        except Exception as exc:
            print(f"[STOP] 정지 명령 실패: {exc} — Pi 워치독이 0.3초 안에 세웁니다")
        for worker in workers:
            worker.stop()
        for cap in caps:
            cap.release()
        cv2.destroyAllWindows()
        if live_map is not None:
            live_map.close()
        if isinstance(link, UdpVehicleLink):
            link.close()

    print(f"\n\n종료 — 마지막 상태: {fsm.state.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
