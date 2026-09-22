"""Host 메인 루프 — 탑뷰 카메라 + ArUco + 검출기 -> 미션 FSM -> Pi(UDP).

사용법 (host/ 에서):
    python run_host.py --sim                          # 차량·카메라 없이 전체 흐름
    python run_host.py --sim --step                   # n 키로 단계 진행
    python run_host.py --pi-ip 192.168.0.7            # 실기
    python run_host.py --pi-ip 192.168.0.7 --detector none --show-cams
    python run_host.py                                 # 카메라만 열고 명령은 콘솔에 찍기(dry run)

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
    ap.add_argument("--show-cams", action="store_true", help="카메라 원본+ArUco 오버레이 창(디버그)")
    ap.add_argument("--step", action="store_true", help="수동 단계 모드(n 키로 진행)")
    ap.add_argument("--hz-every", type=int, default=50, help="N 사이클마다 루프 Hz 출력(0=끔)")
    ap.add_argument("--max-cycles", type=int, default=0, help="0 = 무한")
    args = ap.parse_args()

    if args.sim and args.pi_ip:
        print("--sim 과 --pi-ip 는 같이 쓸 수 없습니다")
        return 2

    try:
        cfg = load_host_config(args.config)
    except host_config.ConfigError as exc:
        print(f"[config] {exc}")
        return 2

    signal.signal(signal.SIGINT, _on_sigint)
    fsm = MissionFSM(cfg, manual_mode=args.step)
    view = None
    caps, cams, piece_detector, world = [], [], None, None

    try:
        if args.sim:
            from sim.sim_world import SimWorld
            world = SimWorld(cfg)
            link = world.link
            print("[host] SIM 모드 — 기물 2개를 상자로 옮기는 흐름을 흉내냅니다")
        else:
            import cv2
            from localization.aruco_localizer import Camera, RobotLocalizer, detect, draw_overlay, make_detector
            from localization.cameras import open_cams, read_frames
            from perception.detector import make_detector as make_piece_detector
            from perception.piece_tracker import PieceTracker, observations_from_detections

            indices = args.cams if args.cams is not None else list(cfg.cameras.indices)
            caps = open_cams(indices, cfg.cameras.width, cfg.cameras.height)
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
            if args.pi_ip:
                link = UdpVehicleLink(args.pi_ip, cfg.link.command_port, cfg.link.status_port,
                                      cfg.link.bind_ip, cfg.link.stop_burst)
                print(f"[host] Pi 링크: -> {args.pi_ip}:{cfg.link.command_port}, "
                      f"상태 수신 :{cfg.link.status_port}")
            else:
                link = ConsoleLink()
                print("[host] --pi-ip 없음 — 명령을 콘솔에만 찍습니다(dry run)")

        if not args.no_view:
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
                                                            cfg.detector.conf_threshold))
                pmap = tracker.update(obs, t0)
                if args.show_cams:
                    for cam, frame, det in zip(cams, frames, dets):
                        if frame is not None:
                            cv2.imshow(cam.name, draw_overlay(frame.copy(), cam, det, pose,
                                                              cfg.aruco.robot_marker_id))

            status = link.latest_status()
            cmd = fsm.step(pose, pmap, status, t0)
            link.send(cmd)

            action = None
            if view is not None:
                action = view.update(pose, pmap, fsm, status, link.status_age_s(), hz)
            elif args.show_cams:
                import cv2 as _cv2
                action = "quit" if (_cv2.waitKey(1) & 0xFF) == ord("q") else None
            if action == "quit":
                break
            if action == "estop":
                fsm.request_estop()
            elif action == "reset":
                fsm.reset()
                if world is None:
                    tracker.reset()
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
            for c in caps:
                c.release()
            if view is not None:
                view.close()
            elif args.show_cams:
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
