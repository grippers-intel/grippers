# domain.values.BoxObservation 의 ROS2 대응. find_box() 응답, align_to_box()/
# measure_opening() 요청에서 재사용한다 — 셋 다 "이 상자" 를 지칭해야 하기 때문.

string color             # BoxColor 이름: "BLACK" | "RED" | "BLUE" | "GREEN"
geometry_msgs/Pose2D pose
float32 opening_mm
float64 long_axis_rad
