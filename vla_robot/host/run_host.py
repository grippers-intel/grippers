"""Host 메인 루프 — 탑뷰 카메라 + ArUco + 검출기 -> 미션 FSM -> Pi(UDP).

손 검출(MediaPipe, `hands:`)은 지금은 **지도 표시만** 한다 — FSM 에는 아직 넘기지 않는다.

사용법 (host/ 에서):
    python run_host.py --sim                          # 차량·카메라 없이 전체 흐름
    python run_host.py --sim --step                   # n 키로 단계 진행
    python run_host.py --pi-ip 192.168.0.7            # 실기
    python run_host.py --pi-ip 192.168.0.7 --detector none --show-cams
    python run_host.py                                 # 카메라만 열고 명령은 콘솔에 찍기(dry run)
    python run_host.py --sim --view cv                 # 예전 OpenCV 지도 창(기본은 팀원 시연 UI)
    python run_host.py --sim --mode instructed --sim-hand L2   # 지시를 기다리는 시뮬, 손은 L2

매 사이클 명령을 **반드시 하나** 보낸다. pose 를 잃었으면 stop 이다(FSM 참고).
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

HOST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402  (vla_common 경로 보정 포함)
from host_config import load_host_config  # noqa: E402
from link.vehicle_link import ConsoleLink, UdpVehicleLink  # noqa: E402
from localization.pose import Pose  # noqa: E402
from mission.host_fsm import HostState, MissionFSM  # noqa: E402

_stop = False


def _on_sigint(_signum, _frame):
    global _stop
    _stop = True


def main() -> int:
    host_config.configure_console()
    ap = argparse.ArgumentParser(description="vla_robot Host")
    ap.add_argument("--config", default=None, help="host.yaml 경로 (기본 host/config/host.yaml)")
    ap.add_argument("--pi-ip", default=None, help="Pi IP. 없으면 --sim 또는 콘솔 dry run")
    ap.add_argument("--sim", action="store_true", help="카메라·차량 없이 SimWorld 로 돌린다")
    ap.add_argument("--cams", type=int, nargs="+", default=None)
    ap.add_argument("--detector", choices=("geti", "none"), default=None)
    ap.add_argument("--geti-device", default=None)
    ap.add_argument("--no-view", action="store_true")
    ap.add_argument("--view", choices=("web", "cv"), default=None,
                    help="web = 팀원 시연 UI(브라우저 앱 창) · cv = 예전 OpenCV 지도. 기본 host.yaml view.kind")
    ap.add_argument("--show-cams", action="store_true", help="카메라 원본+ArUco 오버레이 창(디버그)")
    ap.add_argument("--step", action="store_true", help="수동 단계 모드(n 키로 진행)")
    ap.add_argument("--mode", choices=("auto", "instructed"), default=None,
                    help="auto = 보이는 기물을 모두 정리(지시 우선) · instructed = 지시가 있을 때만. 기본 host.yaml instruction.mode")
    ap.add_argument("--sim-hand", nargs="*", default=[], metavar="SPOT",
                    help="--sim 에서 손을 둘 위치 이름(F1~R3). 예: --sim-hand L2")
    ap.add_argument("--hz-every", type=int, default=50, help="N 사이클마다 루프 Hz 출력(0=끔)")
    ap.add_argument("--max-cycles", type=int, default=0, help="0 = 무한")
    ap.add_argument("--log-piece", default=None, metavar="LABEL",
                    help="이 라벨(쉼표로 여러 개, all = 전부)의 카메라별 위치를 1 s 마다 찍는다"
                         "(넓은 기물 반경 재기, 10-05)")
    args = ap.parse_args()

    if args.sim and args.pi_ip:
        print("--sim 과 --pi-ip 는 같이 쓸 수 없습니다")
        return 2

    try:
        cfg = load_host_config(args.config)
    except host_config.ConfigError as exc:
        print(f"[config] {exc}")
        return 2

    if args.mode:
        from dataclasses import replace
        cfg = replace(cfg, instruction=replace(cfg.instruction, mode=args.mode))
    signal.signal(signal.SIGINT, _on_sigint)
    fsm = MissionFSM(cfg, manual_mode=args.step)
    view, cv_view = None, False
    caps, cams, piece_detector, world = [], [], None, None
    hand_detector, hand_tracker, hands, hand_spots = None, None, [], []
    piece_log_at = 0.0                     # --log-piece 마지막 출력 시각

    try:
        if args.sim:
            from sim.sim_world import SimWorld
            unknown = [s for s in args.sim_hand if s not in cfg.handover.spots]
            if unknown:
                print(f"--sim-hand: 모르는 위치 {unknown} (가능: {', '.join(cfg.handover.spots)})")
                return 2
            world = SimWorld(cfg, hands=[cfg.handover.spots[s][:2] for s in args.sim_hand])
            link = world.link
            print("[host] SIM 모드 — 기물 2개를 상자로 옮기는 흐름을 흉내냅니다")
        else:
            import cv2
            from localization.aruco_localizer import Camera, RobotLocalizer, detect, draw_overlay, make_detector
            from localization.cameras import open_cams, read_frames
            from perception.detector import make_detector as make_piece_detector
            from perception.hands import (HandDetector, hand_observations, make_hand_tracker,
                                          nearest_spot)
            from perception.piece_tracker import PieceTracker, observations_from_detections

            indices = args.cams if args.cams is not None else list(cfg.cameras.indices)
            caps = open_cams(indices, cfg.cameras.width, cfg.cameras.height, cfg.cameras.focus)
            if not any(c.isOpened() for c in caps):
                print("[host] 열린 카메라가 없습니다. --cams 로 인덱스를 바꿔 보세요")
                return 1
            cams = [Camera.load(i, cfg.cameras, cfg.aruco) for i in indices]
            aruco_detector = make_detector(cfg.aruco)
            localizer = RobotLocalizer(cfg.aruco)
            det_cfg = cfg.detector
            if args.detector or args.geti_device:
                from dataclasses import replace
                det_cfg = replace(det_cfg, kind=args.detector or det_cfg.kind,
                                  device=args.geti_device or det_cfg.device)
            print(f"[host] 검출기: {det_cfg.kind}")
            piece_detector = make_piece_detector(det_cfg, indices)
            tracker = PieceTracker(cfg.tracker)
            hand_detector = HandDetector(cfg.hands, indices)
            hand_tracker = make_hand_tracker(cfg.hands, cfg.tracker)
            if hand_detector.ok:
                print(f"[host] 손 검출: MediaPipe · 높이 {cfg.hands.hand_z_m} m · "
                      f"가장자리 {'/'.join(cfg.hands.edges)} {cfg.hands.edge_band_m} m")
            if args.pi_ip:
                link = UdpVehicleLink(args.pi_ip, cfg.link.command_port, cfg.link.status_port,
                                      cfg.link.bind_ip, cfg.link.stop_burst)
                print(f"[host] Pi 링크: -> {args.pi_ip}:{cfg.link.command_port}, "
                      f"상태 수신 :{cfg.link.status_port}")
            else:
                link = ConsoleLink()
                print("[host] --pi-ip 없음 — 명령을 콘솔에만 찍습니다(dry run)")

        cv_view = not args.no_view and (args.view or cfg.view.kind) == "cv"
        if not args.no_view:
            if not cv_view:
                from view.web_view import WebView
                view = WebView(cfg)
            else:
                from view.map_view import MapView
                view = MapView(cfg)

        period = 1.0 / cfg.mission.cycle_hz
        hz, hz_n, hz_t0, cycles = 0.0, 0, time.monotonic(), 0
        pose, pmap = Pose(), {}

        while not _stop:
            t0 = time.monotonic()
            if world is not None:
                world.update()
                pose, pmap = world.pose(), world.piece_map()
                hands = list(world.hands)
            else:
                frames = read_frames(caps)
                dets = [{} if f is None else detect(aruco_detector, cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
                        for f in frames]
                pose = localizer.update(cams, dets)
                obs = []
                for idx, cam, frame in zip(indices, cams, frames):
                    if frame is not None:
                        piece_detector.submit(idx, frame)
                    obs.append(observations_from_detections(cam, piece_detector.latest(idx),
                                                            cfg.detector.conf_threshold,
                                                            cfg.tracker.piece_radius_by_label))
                pmap = tracker.update(obs, t0)
                if args.log_piece and t0 - piece_log_at >= 1.0:
                    piece_log_at = t0
                    want = args.log_piece.split(",")
                    labels = sorted({o.label for lst in obs for o in lst} | set(pmap)) if want == ["all"] else want
                    for label in labels:
                        seen = [f"{o.cam_name} ({o.x:.3f},{o.y:.3f})" for lst in obs for o in lst
                                if o.label == label]
                        fused = [f"({x:.3f},{y:.3f})" for x, y in pmap.get(label, [])]
                        # 두 카메라가 같은 기물을 보면 반경 추정: 각자 자기 쪽 가장자리를 보므로
                        # p_i = 중심 + r * (카메라 i 쪽 단위벡터) -> r = |p0 - p1| / |u0 - u1| (장판 어디서든)
                        est = ""
                        per = {o.cam_name: o for lst in obs for o in lst if o.label == label}
                        named = {c.name: c for c in cams if getattr(c, "center", None) is not None}
                        if len(per) == 2 and set(per) <= set(named):
                            (n0, o0), (n1, o1) = sorted(per.items())
                            us = []
                            for n, o in ((n0, o0), (n1, o1)):
                                cx, cy = float(named[n].center[0]), float(named[n].center[1])
                                dx, dy = cx - o.x, cy - o.y
                                k = (dx * dx + dy * dy) ** 0.5 or 1.0
                                us.append((dx / k, dy / k))
                            du = ((us[0][0] - us[1][0]) ** 2 + (us[0][1] - us[1][1]) ** 2) ** 0.5
                            gap = ((o0.x - o1.x) ** 2 + (o0.y - o1.y) ** 2) ** 0.5
                            if du > 0.3:
                                est = f" · 차이 {gap * 100:.1f} cm -> 반경 추정 {gap / du:.3f} m"
                        print(f"[piece] {label} 카메라별 " + (" · ".join(seen) or "없음")
                              + " -> 지도 " + (" ".join(fused) or "없음") + est, flush=True)
                hobs = []
                if hand_detector.ok:
                    for idx, frame in zip(indices, frames):
                        if frame is not None:
                            hand_detector.submit(idx, frame)
                    hobs = hand_observations(cams, [hand_detector.latest(i) for i in indices],
                                             cfg.hands, cfg.arena)
                hands = hand_tracker.update([hobs], t0).get("hand", [])
                spots = sorted(nearest_spot(h) for h in hands)
                if spots != hand_spots:     # 손이 생기거나 사라지거나 자리를 옮길 때만 찍는다
                    print("[hands] " + (", ".join(f"{nearest_spot(h)} ({h[0]:.2f},{h[1]:.2f})"
                                                  for h in hands) or "손 없음"))
                    hand_spots = spots
                if args.show_cams:
                    for idx, cam, frame, det in zip(indices, cams, frames, dets):
                        if frame is not None:
                            over = draw_overlay(frame.copy(), cam, det, pose, cfg.aruco.robot_marker_id)
                            for s in (hand_detector.latest(idx) or []) if hand_detector.ok else []:
                                c = tuple(int(v) for v in s.palm)
                                cv2.circle(over, c, 10, (0, 140, 255), 2)
                            cv2.imshow(cam.name, over)

            status = link.latest_status()
            fsm.set_hands(hands)
            cmd = fsm.step(pose, pmap, status, t0)
            link.send(cmd)

            action = None
            if view is not None:
                action = view.update(pose, pmap, fsm, status, link.status_age_s(), hz, hands)
            if args.show_cams and not cv_view:
                # 카메라 창은 OpenCV 라 waitKey 가 돌아야 그려진다. cv 지도는 자기가 부른다.
                import cv2 as _cv2
                key = _cv2.waitKey(1) & 0xFF
                if view is None and key == ord("q"):
                    action = "quit"
            if action == "quit":
                break
            if action == "estop":
                fsm.request_estop()
            elif action == "reset":
                fsm.reset()
                if world is None:
                    tracker.reset()
                    hand_tracker.reset()
            elif action == "next":
                fsm.request_advance()
            elif action == "prev":
                fsm.request_back()
            elif action == "toggle_manual":
                fsm.set_manual_mode(not fsm.manual_mode)
                print(f"[host] 모드 -> {'MANUAL' if fsm.manual_mode else 'AUTO'} (reset)")

            cycles += 1
            hz_n += 1
            if hz_n >= 20:
                now = time.monotonic()
                hz = hz_n / max(now - hz_t0, 1e-6)
                hz_n, hz_t0 = 0, now
            if args.hz_every and cycles % args.hz_every == 0:
                print(f"[host] {hz:5.1f} Hz  {fsm.state.name:15s} {pose}  cmd={fsm.last_cmd_text}")
            if args.max_cycles and cycles >= args.max_cycles:
                break
            if world is not None and fsm.state == HostState.SEARCH_TARGET and not pmap_in_workspace(cfg, pmap):
                if cycles % 50 == 0:
                    print("[host] SIM: 옮길 기물이 더 없습니다 (q 로 종료)")
            remaining = period - (time.monotonic() - t0)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        try:
            if "link" in locals():
                link.close()
        finally:
            if piece_detector is not None:
                piece_detector.close()
            if hand_detector is not None:
                hand_detector.close()
            for c in caps:
                c.release()
            if view is not None:
                view.close()
            if args.show_cams and not cv_view:
                import cv2 as _cv2
                _cv2.destroyAllWindows()
    print(f"[host] 종료 — 마지막 상태 {fsm.state.name}")
    return 0


def pmap_in_workspace(cfg, pmap) -> bool:
    a = cfg.arena
    return any(a.workspace_x[0] <= x <= a.workspace_x[1] and a.workspace_y[0] <= y <= a.workspace_y[1]
               for pts in pmap.values() for x, y in pts)


if __name__ == "__main__":
    sys.exit(main())
