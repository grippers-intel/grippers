#!/usr/bin/env bash
# 파지 자세 교시 — 호스트에서 실행한다.
#   ./teach approach     현재 팔 자세를 'approach' 라는 이름으로 저장
#   ./teach --list       저장된 자세 목록
# 텔레옵이 돌고 있어야 한다(팔로워 노드가 관절값을 발행하므로).
exec docker exec IntelPi bash -lc \
  "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=21 &&
   python3 -u /grippers/tools/teach/teach_pose.py $*"
