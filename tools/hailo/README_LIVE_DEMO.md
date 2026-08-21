# Hailo-10H 실시간 YOLO 검출 데모 (`live_yolo_demo.py`)

`hld.md`에 "별도 검증 항목으로 유지"라고 명시돼 있던 ④ HailoRT 추론 검증을
실기에서 처음 통과시킨 탐색용 스크립트다 (2026-08-21). **프로덕션
`domain/ports/perception.py` 어댑터가 아니다** — 편입하려면 별도 작업이
필요하다.

## 확인된 것 (실기, 2026-08-21)

- `hailortcli parse-hef` — "HEF Compatible for: HAILO15H, HAILO10H"
- `hailortcli run2` — 물리 Hailo-10H PCIe 장치에서 **230 FPS**
- depth_cam(회전 보정)·gripper_cam 두 소스 모두 같은 VDevice/모델을 공유해
  동시 추론 — 물리 장치가 1개뿐이라 카메라마다 별도 프로세스를 띄우면
  `HAILO_OUT_OF_PHYSICAL_DEVICES`로 죽는다(스크립트 상단 주석 참고)

## 실행 전제

- HEF 파일이 필요하다 — **git에 안 들어있다** (vendor 바이너리와 같은
  이유, `docker/vendor/README.md` 관례 참고). 기본 경로는
  `/tmp/best_640.hef`, `--ros-args -p hef_path:=...`로 바꿀 수 있다.
- `depth_cam_rotate_node`(depth cam 180도 회전 보정)와
  `gripper_cam_publisher_node`(그리퍼캠 raw V4L2 → ROS2 Image 브리지)가
  먼저 떠 있어야 한다 — 둘 다 `grippers_perception` 패키지 소속.
- `perception_node`의 `confirm_grasp` 서비스는 `/dev/gripper_cam`을
  독점하려고 하므로 `gripper_cam_publisher_node`와 동시 실행 금지.

## 실행

```bash
# 카메라 소스 (각각 별도 프로세스, 카메라별 장치이므로 무관)
ros2 run grippers_perception depth_cam_rotate_node
ros2 run grippers_perception gripper_cam_publisher_node

# YOLO 데모 (VDevice 1개 공유, 두 소스 동시 구독)
python3 tools/hailo/live_yolo_demo.py
```

결과 토픽 `depth_cam/yolo/image_detections`, `gripper_cam/yolo/image_detections`는
`web_video_server`로 HTTP MJPEG 스트리밍해서 확인했다(맥북에 X11/rqt 없이
ffplay로 봄) — 이 프로젝트에 X11 forwarding 인프라가 없다면 같은 방식을
권장한다.

## 알려진 제약

- 입력을 640x640으로 레터박싱하고, **좌표 역변환 없이 레터박스된 프레임
  자체에 박스를 그려 퍼블리시**한다 — 원본 프레임 좌표계가 아니다.
- `SCORE_THRESHOLD=0.35`, `CLASS_NAMES` 순서는 `best_hailo10h_640/metadata.yaml`의
  `names`와 반드시 일치해야 한다 — 다른 HEF로 바꾸면 같이 바꿀 것.
