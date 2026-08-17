# domain.values.Detection 의 ROS2 대응 (docs/design/class_diagram.md §1).
# 단위 규약: pose/dims는 m, yaw_rad는 rad — 필드명에 그대로 반영.

uint32 track_id
string cls              # ObjectClass 이름: "GABE" | "CHESS_PIECE"
geometry_msgs/Point pose
geometry_msgs/Vector3 dims
float64 yaw_rad
float32 confidence
