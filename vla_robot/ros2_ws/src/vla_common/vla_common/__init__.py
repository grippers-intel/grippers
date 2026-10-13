"""Host 와 Pi 가 공유하는 계약.

이 패키지는 rclpy·cv2·torch 를 최상위에서 import 하지 않는다. Windows Host 와
ROS 컨테이너 양쪽에서 똑같이 로드되어야 하기 때문이다(torch 는
`policy_runner` 안에서만 늦게 불러온다).
"""
