# domain.values.BoxObservation 의 ROS2 대응. find_box() 응답, align_to_box()/
# measure_opening() 요청에서 재사용한다 — 셋 다 "이 상자" 를 지칭해야 하기 때문.
#
# ⚠️ 2026-08-23: domain.values.BoxColor가 Destination(LEFT/RIGHT)으로
# 바뀌었다(확정 미션 명세서 — 좌표 하드코딩, 색 탐색 안 함). 필드명 `color`는
# 인터페이스 재빌드 없이 이번 변경 범위 밖에 남겨뒀다 — 지금은 이 문자열
# 필드에 Destination 이름을 담는다. domain/adapters/real/_ros_convert.py 참고.

string color             # Destination 이름: "LEFT" | "RIGHT"
geometry_msgs/Pose2D pose
float32 opening_mm
float64 long_axis_rad
