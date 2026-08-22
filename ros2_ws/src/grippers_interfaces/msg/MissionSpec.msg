# domain.values.MissionSpec 의 ROS2 대응.
#
# placement_rule(dict[ObjectClass, Destination])은 .msg가 dict를 지원하지 않아
# 병렬 배열 2개로 평탄화한다 — class_diagram.md §1의 미결 사항
# ("배포판 확정(#1) 이후 결정")을 여기서 해소한다. placement_classes[i]가
# placement_colors[i]로 간다. 두 배열은 항상 길이가 같아야 한다.
#
# ⚠️ 2026-08-23: domain.values.BoxColor가 Destination(LEFT/RIGHT)으로
# 바뀌었다(확정 미션 명세서 — 좌표 하드코딩, 색 탐색 안 함). placement_colors는
# 이제 색 이름이 아니라 Destination 이름("LEFT"/"RIGHT")을 담는다. 필드명
# 자체(`placement_colors`)를 바꾸는 건 인터페이스 재빌드가 필요해 팀과
# 별도로 조율할 것 — domain/adapters/real/ros2_command_interpreter.py 참고.

string mode                     # MissionMode 이름: "TIDY" | "FETCH"
string target_cls               # ObjectClass 이름. TIDY처럼 대상이 없으면 빈 문자열
string[] placement_classes
string[] placement_colors
string raw_text
