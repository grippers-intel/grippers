"""학습된 정책을 실기에 돌린다 — 성능 시험이 아니라 배포 경로 검증용.

## 왜 필요한가

정책이 계산을 한다는 것과 **그 계산으로 팔이 움직인다**는 것은 다르다.
체크포인트를 옮기고 forward 시간을 재고 MAE 를 대조해도, 카메라 → 전처리 →
정책 → 서보로 이어지는 배선이 끝까지 도는지는 확인되지 않는다.

이 스크립트가 확인하는 것은 하나다.

    제어 루프가 끝까지 도는가

팔이 과제를 성공하면 보너스다. 못 해도 루프가 돌면 목적을 달성한 것이다.

## 무엇을 보는가 — forward 가 루프를 막는다

`select_action` 은 `n_action_steps` 마다 한 번만 forward 를 돌고, 그동안
**제어 루프가 통째로 멈춘다.** 그래서 forward 시간을 청크 수명과 비교해야 한다.

    청크 수명 = n_action_steps / fps          (100스텝 / 30fps = 3.33초)

forward 가 청크 수명을 넘으면 다음 청크가 나오기 전에 이전 청크가 소진된다.
느린 게 아니라 **팔이 단속적으로 움직인다** — 실측(2026-08-31, act_queen_v3 015000):

    노트북 x86  720p    795 ms   루프 23.9 Hz (목표 30)  forward 1회당 22스텝 손실
    Pi aarch64  720p   4130 ms   청크의 124%  →  3.3초 이동 / 4.1초 정지 반복
    Pi aarch64  480x640 1515 ms  45%
    Pi aarch64  240x320  951 ms  29%

Pi 에서 720p 는 쓸 수 없다. 해상도를 낮추거나 `lerobot.async_inference` 로
forward 를 루프 밖으로 빼야 한다.

## 회전 계약

그리퍼캠은 거꾸로 달려 있다. LeRobot 경로는 카메라 설정의 `rotation: 180` 으로
맞추고, ROS 경로는 `gripper_cam_geometry.orient()` 로 맞춘다. 둘은 같은 것을
가리켜야 한다 — 자세한 이유는 그 파일의 docstring 에 있다.

    학습 때 정책이 본 화면  ==  추론 때 정책이 보는 화면

## 안전장치

`--max-rel` 이 유일한 안전장치다. 한 스텝에 움직일 수 있는 크기를 제한한다.
LeRobot 기본값은 `None`(무제한)이고, **int 를 주면 TypeError 로 죽는다**
(`robots/utils.py` 가 `isinstance(x, float)` 로 검사한다). 여기서는 float 로
넘긴다. 그 위에 관절 한계 검사를 얹어, 벗어나는 값이 나오면 보내기 전에 멈춘다.

## 쓰는 법

    python tools/arm/rollout_policy.py --ckpt <pretrained_model 경로> [--seconds 20]

포트와 카메라는 자동으로 찾는다. 6축이 응답하는 시리얼 포트와, 열리는 첫
카메라를 고른다. 여러 개면 `--port`, `--camera` 로 직접 지정할 것.

⚠️ 팔이 실제로 움직인다. 작업 공간을 비우고 전원을 끊을 수 있게 두고 돌릴 것.
"""

import argparse
import os
import platform
import sys
import time

os.environ.setdefault("HF_HUB_OFFLINE", "1")

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

#: 보내기 전에 걸러내는 관절 한계(도). 서보 가동범위보다 넉넉하게 잡되,
#: 학습 분포 밖 입력으로 정책이 튀었을 때 잡히도록 둔다.
JOINT_LIMITS = {
    "shoulder_pan": (-90, 90),
    "shoulder_lift": (-115, 100),
    "elbow_flex": (-40, 110),
    "wrist_flex": (-95, 95),
    "wrist_roll": (-180, 180),
    "gripper": (-5, 105),
}


def find_port(explicit: str | None) -> str:
    """6축이 응답하는 시리얼 포트를 찾는다. Windows/Linux 양쪽."""
    import scservo_sdk as scs
    from serial.tools import list_ports

    if explicit:
        return explicit

    candidates = [p.device for p in list_ports.comports() if "Bluetooth" not in p.description]
    print("시리얼 후보:", candidates or "(없음)")
    for dev in candidates:
        ph = scs.PortHandler(dev)
        pk = scs.PacketHandler(0)
        try:
            if not (ph.openPort() and ph.setBaudRate(1_000_000)):
                continue
            n = sum(1 for sid in range(1, 7) if pk.ping(ph, sid)[1] == scs.COMM_SUCCESS)
        finally:
            ph.closePort()
        print(f"  {dev}: 서보 {n}개 응답")
        if n == 6:
            return dev
    sys.exit("6축이 응답하는 포트를 못 찾았습니다. 전원과 케이블을 확인하세요.")


def find_camera(explicit: int | None, width: int, height: int) -> tuple[int, int]:
    """열리는 카메라 index 와 그 플랫폼의 OpenCV 백엔드를 돌려준다.

    Windows 는 MSMF 를 먼저 보고, 안 열리면 DSHOW 로 떨어진다. Linux 는 V4L2.
    DSHOW 도 MJPG 30fps 가 나온다 — 단 해상도를 FOURCC 보다 먼저 걸어야 한다
    (patch_dshow_property_order 참고). "DSHOW 는 느리다"는 통념은 순서 문제였다.
    """
    import cv2
    from lerobot.cameras.configs import Cv2Backends

    if platform.system() == "Windows":
        # MSMF 를 먼저 본다. 그런데 이 백엔드는 드라이버 상태에 따라 통째로
        # 안 열릴 때가 있다(2026-09-02 실측: MSMF 로는 index 0~7 이 전부
        # isOpened()=False, 같은 카메라가 DSHOW 로는 열림). 그때 죽지 않고
        # DSHOW 로 떨어진다 — 대신 아래 patch_dshow_property_order() 가 필요하다.
        candidates = [(Cv2Backends.MSMF, cv2.CAP_MSMF), (Cv2Backends.DSHOW, cv2.CAP_DSHOW)]
    else:
        candidates = [(Cv2Backends.V4L2, cv2.CAP_V4L2)]

    if explicit is not None:
        return explicit, candidates[0][0]

    for backend, cv_backend in candidates:
        for idx in range(8):
            cap = cv2.VideoCapture(idx, cv_backend)
            try:
                if not cap.isOpened():
                    continue
                # DSHOW 는 이 순서여야 MJPG 로 협상된다(patch 주석 참고).
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                ok, frame = cap.read()
                # 죽은 장치(Phone Link 같은 가상 카메라)는 열리고 읽히지만 단색
                # 플레이스홀더를 준다. 그걸 그리퍼캠으로 고르면 롤아웃이 통째로 무의미하다.
                if ok and frame is not None and frame.std() > 8.0:
                    print(f"  카메라 index {idx}: {frame.shape[1]}x{frame.shape[0]}"
                          f"  ({backend.name})")
                    return idx, backend
            finally:
                cap.release()
            time.sleep(0.4)
        print(f"  {backend.name}: 쓸 수 있는 카메라 없음")
    sys.exit("열리는 카메라를 못 찾았습니다.")


def patch_dshow_property_order() -> None:
    """DSHOW 에서 해상도를 FOURCC 보다 **먼저** 걸도록 LeRobot 을 고쳐 끼운다.

    LeRobot 은 FOURCC 를 먼저 건다(camera_opencv.py:202, "FOURCC first as it can
    affect available FPS/resolution options"). MSMF·V4L2 에서는 맞는 순서지만
    DSHOW 에서는 정확히 거꾸로다. 2026-09-02 실측(index 0, 1280x720):

        해상도 -> FOURCC      MJPG   29.9 fps
        FOURCC -> 해상도      YUY2   10.0 fps      <- LeRobot 의 순서
        해상도만              YUY2   10.0 fps      <- fourcc=None 일 때

    10fps 는 정책을 못 쫓아가게 만든다 — 관측이 낡은 채로 추론하고, max_rel 이
    그 지연을 다시 키운다. 그래서 백엔드가 DSHOW 일 때만 미리 해상도를 걸어
    두고 원래 함수를 부른다(원래 함수가 FOURCC 뒤에 해상도를 한 번 더 걸지만
    같은 값이라 협상을 되돌리지 않는다 — 실측으로 확인).
    """
    import cv2
    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

    if getattr(OpenCVCamera, "_dshow_order_patched", False):
        return
    original = OpenCVCamera._configure_capture_settings

    def configure(self):
        if self.backend == cv2.CAP_DSHOW and self.config.width and self.config.height:
            self.videocapture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.config.width))
            self.videocapture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.config.height))
        return original(self)

    OpenCVCamera._configure_capture_settings = configure
    OpenCVCamera._dshow_order_patched = True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True, help="pretrained_model 디렉터리")
    ap.add_argument("--id", default="grippers_arm", help="캘리브레이션 id")
    ap.add_argument("--port", default=None)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--res", nargs=2, type=int, default=(720, 1280), metavar=("H", "W"))
    ap.add_argument("--task", default="pick up the queen")
    ap.add_argument("--max-rel", type=float, default=5.0, help="스텝당 최대 이동(도). 유일한 안전장치")
    ap.add_argument("--policy-res", nargs=2, type=int, default=None, metavar=("H", "W"),
                    help="정책에 넣기 전 소프트웨어 리사이즈. 생략하면 train_config.json 에서 자동 판별")
    ap.add_argument("--n-action-steps", type=int, default=None,
                    help="청크당 재예측 없이 쓰는 스텝 수. 줄이면 개루프 드리프트가 준다")
    ap.add_argument("--goal-speed", type=int, default=0,
                    help="서보 Goal_Speed(raw/s). 0=무제한. 앞선 도구가 남긴 값을 덮는다")
    ap.add_argument("--freeze-wrist-roll", type=float, default=0.067,
                    help="정책에 넣을 wrist_roll 상수. v5 학습 평균이 기본값")
    args = ap.parse_args()

    # 모터 통신 재시도. LeRobot 기본값이 0 이라 패킷 하나 어긋나면 죽는다 —
    # 롤아웃 도중에 죽으면 팔이 중간 자세로 남는다. feetech_retry.py 참고.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from feetech_retry import patch_bus
    patch_bus()

    import torch
    from lerobot.cameras.configs import Cv2Backends, Cv2Rotation
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
    from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
    from lerobot.robots.so_follower.so_follower import SOFollower

    h, w = args.res

    # 학습에 쓴 리사이즈는 train_config.json 에 있다. config.json 은 데이터셋
    # 원본 크기를 적으므로 못 쓴다(policies/factory.py 가 ds_meta.features 를 읽는다).
    #
    # ⚠️ --res 는 **카메라에 요청할 촬영 해상도**다. 여기에 180x320 을 주면
    # 리사이즈가 아니라 다른 카메라 모드(다른 화각)가 온다 - 이 카메라는 4:3 을
    # 요청하면 좌우를 25% 잘라낸다. 학습은 1280x720 으로 찍은 것을 소프트웨어로
    # 줄인 것이므로, 촬영은 원본으로 하고 여기서 줄인다.
    ph, pw = (args.policy_res if args.policy_res else (None, None))
    if ph is None:
        import json as _json
        try:
            _tf = _json.load(open(os.path.join(args.ckpt, "train_config.json"),
                                  encoding="utf-8"))["dataset"]["image_transforms"]
            if _tf.get("enable") and "resize" in _tf.get("tfs", {}):
                ph, pw = _tf["tfs"]["resize"]["kwargs"]["size"]
                print(f"학습 리사이즈 자동 판별: {ph}x{pw}  (train_config.json)")
        except Exception as _e:
            print(f"train_config.json 을 못 읽었습니다({_e}) - 원본 크기로 넣습니다")
    if ph is None:
        ph, pw = h, w

    port = find_port(args.port)
    cam_idx, backend = find_camera(args.camera, w, h)
    print(f"\n선택: 포트 {port}  카메라 {cam_idx}  {w}x{h}@{args.fps}\n")

    # LeRobot 0.4.4 는 config 에 `type` 이 있어야 하고(`policies.py` 가 pop 한다),
    # `pretrained_revision` 은 받지 않는다. 진입점을 PreTrainedConfig 로 두면
    # draccus 가 `type` 을 판별자로 소비한다.
    cfg = PreTrainedConfig.from_pretrained(args.ckpt)
    cfg.device = "cpu"
    if args.n_action_steps:
        print(f"n_action_steps {cfg.n_action_steps} -> {args.n_action_steps} "
              f"(재예측 주기 {args.n_action_steps / args.fps:.2f}초)")
        cfg.n_action_steps = args.n_action_steps
    policy = ACTPolicy.from_pretrained(args.ckpt, config=cfg)
    policy.to("cpu").eval()
    # 학습 때 device(xpu)가 전처리기에 박혀 있다. 안 덮으면 mat1 is on xpu:0 로 죽는다.
    pre, post = make_pre_post_processors(
        cfg, pretrained_path=args.ckpt,
        preprocessor_overrides={"device_processor": {"device": "cpu"}},
    )

    if backend == Cv2Backends.DSHOW:
        patch_dshow_property_order()
    cam = OpenCVCameraConfig(index_or_path=cam_idx, fps=args.fps, width=w, height=h,
                             rotation=Cv2Rotation.ROTATE_180, backend=backend,
                             fourcc="MJPG")
    robot = SOFollower(SO101FollowerConfig(
        port=port, id=args.id, max_relative_target=float(args.max_rel),
        disable_torque_on_disconnect=False, cameras={"gripper": cam}))
    robot.connect(calibrate=False)   # 기존 캘리브레이션을 쓴다. True 면 덮어쓸 수 있다.

    # Goal_Velocity(주소 46, align_to_idle 이 "speed" 로 부르는 그 레지스터)를
    # 여기서 정한다 — 앞서 무엇이 돌았든 상관없게.
    #
    # LeRobot 의 configure() 는 Acceleration 은 254 로 되돌리지만 이 레지스터는
    # 건드리지 않는다. 그래서 align_to_idle(SPEED_RAW=150, 사람이 지켜보며 도는
    # 정렬이라 일부러 느리다)이 남긴 값이 서보 RAM 에 그대로 남아 롤아웃을 묶는다.
    # 2026-09-02 실측: 정책이 shoulder_lift 를 50도로 부르는데 팔은 스텝당 0.44도,
    # 즉 12.1도/s 로만 갔다 — 150 raw/s = 13.2도/s 와 맞는다.
    #
    # 느린 팔은 그냥 느린 게 아니다. max_relative_target 이 **현재 위치** 기준으로
    # 자르므로, 못 따라간 만큼 다음 목표가 더 잘리고 그게 다시 뒤처짐을 키운다.
    # 0 = 무제한이고, 수집 때 팔로워가 리더를 쫓던 조건도 그것이다. 한 스텝의
    # 이동량은 max_relative_target 이 이미 묶고 있으므로 여기서 또 묶을 이유가 없다.
    for motor in robot.bus.motors:
        robot.bus.write("Goal_Velocity", motor, args.goal_speed)
    print(f"Goal_Velocity <- {args.goal_speed}" + ("  (0=무제한)" if not args.goal_speed else ""))
    print("is_calibrated:", robot.bus.is_calibrated)
    if not robot.bus.is_calibrated:
        print("  ⚠️ 서보 값과 캘리브레이션 파일이 다릅니다. 관절값 해석이 어긋날 수 있습니다.")
    policy.reset()

    chunk_s = cfg.n_action_steps / args.fps
    fwd, loop, stop, step = [], [], None, 0
    t_start = time.perf_counter()
    try:
        while time.perf_counter() - t_start < args.seconds:
            t0 = time.perf_counter()
            obs = robot.get_observation()
            state = torch.tensor([obs[f"{j}.pos"] for j in JOINTS], dtype=torch.float32)
            if args.freeze_wrist_roll is not None:
                # wrist_roll 캘리브레이션 범위가 14틱(2040~2054)뿐이라 1틱만 어긋나도
                # 정규화값이 14 뛴다. 학습 std 가 0.1022 이므로 4틱이면 z=558 이다.
                # 실측: 같은 이미지에 이 값 하나만 바꾸면 정책이 프레임과 무관하게
                # 동일한 액션을 뱉는다 - 트랜스포머가 포화돼 이미지를 무시한다.
                # 학습 평균으로 덮으면 정상 출력과 0.05도 이내로 복구된다.
                state[4] = args.freeze_wrist_roll
            img = torch.from_numpy(obs["gripper"].copy()).permute(2, 0, 1).float() / 255.0
            if (ph, pw) != (img.shape[1], img.shape[2]):
                img = torch.nn.functional.interpolate(
                    img.unsqueeze(0), size=(ph, pw), mode="bilinear",
                    align_corners=False).squeeze(0)
            # 큐가 비어 있는 스텝에서만 실제로 forward 가 돈다(modeling_act.py:116).
            # 소요 시간으로 가르면 안 된다 — 200ms 는 파이 기준이라 노트북에서는
            # 진짜 forward 도 그 아래로 들어와 1회로 세어졌다(2026-09-02).
            refilled = len(policy._action_queue) == 0
            tf = time.perf_counter()
            with torch.no_grad():
                act = post(policy.select_action(pre({
                    "observation.state": state.unsqueeze(0),
                    "observation.images.gripper": img.unsqueeze(0),
                    "task": [args.task]}))).squeeze(0).float()
            if refilled:
                fwd.append((time.perf_counter() - tf) * 1000)

            if not torch.isfinite(act).all():
                stop = "NaN/Inf 출력"
                break
            bad = [f"{j}={float(act[i]):.1f}" for i, j in enumerate(JOINTS)
                   if not (JOINT_LIMITS[j][0] <= float(act[i]) <= JOINT_LIMITS[j][1])]
            if bad:
                stop = "관절 한계 초과: " + ", ".join(bad)
                break

            robot.send_action({f"{j}.pos": float(act[i]) for i, j in enumerate(JOINTS)})
            step += 1
            loop.append(time.perf_counter() - t0)
            time.sleep(max(0.0, 1 / args.fps - (time.perf_counter() - t0)))
    finally:
        final = robot.get_observation()
        robot.disconnect()

    elapsed = time.perf_counter() - t_start
    print("\n=== 결과 ===")
    print(f"중단 사유 : {stop or '없음 (정상 완료)'}")
    print(f"스텝      : {step} / {elapsed:.1f}초  →  {step / elapsed:.1f} Hz "
          f"(목표 {args.fps}, 이상적 {int(args.fps * elapsed)}스텝)")
    if loop:
        s = sorted(loop)
        print(f"루프 주기 : 중앙 {s[len(s) // 2] * 1000:.1f} ms   최대 {s[-1] * 1000:.0f} ms")
    if fwd:
        f = sorted(fwd)
        med = f[len(f) // 2]
        blocked = sum(fwd) / 1000
        print(f"forward   : {len(f)}회  중앙 {med:.0f} ms  최대 {f[-1]:.0f} ms")
        print(f"            청크 수명 {chunk_s:.2f}초 대비 {100 * med / 1000 / chunk_s:.0f}%"
              f"{'  ← 청크를 못 따라감' if med / 1000 > chunk_s else ''}")
        print(f"듀티      : forward 로 멈춘 시간 {blocked:.1f}초 / 전체 {elapsed:.1f}초"
              f"  →  움직인 시간 {100 * (1 - blocked / elapsed):.0f}%")
    print(f"\n{'관절':<15}{'최종':>9}")
    for j in JOINTS:
        print(f"{j:<15}{final[f'{j}.pos']:9.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
