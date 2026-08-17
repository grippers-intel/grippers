# domain.values.MissionSpec 의 ROS2 대응.
#
# placement_rule(dict[ObjectClass, BoxColor])은 .msg가 dict를 지원하지 않아
# 병렬 배열 2개로 평탄화한다 — class_diagram.md §1의 미결 사항
# ("배포판 확정(#1) 이후 결정")을 여기서 해소한다. placement_classes[i]가
# placement_colors[i]로 간다. 두 배열은 항상 길이가 같아야 한다.

string mode                     # MissionMode 이름: "TIDY" | "FETCH"
string target_cls               # ObjectClass 이름. TIDY처럼 대상이 없으면 빈 문자열
string[] placement_classes
string[] placement_colors
string raw_text
