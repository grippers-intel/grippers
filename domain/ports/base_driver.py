"""BaseDriver 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_base의 Ros2MecanumBase가 이걸 구현한다.

⚠️ 2026-08-26 팀 확정으로 이 포트에서 **좌표가 사라졌다.** 예전에는
`drive_to`·`approach`·`align_to_box`가
있었는데, 셋 다 "어디로 갈지 Pi가 계산한다"는 전제 위에 있었다. 이제 물체
좌표, 차량 좌표와 방향, 경로 계산, 차량 제어 명령은 전부 Host가 소유한다.

남은 것은 "받은 속도를 낸다"와 "멈춘다"뿐이다. 여기에 좌표를 받는 메서드를
다시 넣고 싶어지면, 그건 Host의 일이 Pi로 새는 신호다."""

from abc import ABC, abstractmethod


class BaseDriver(ABC):
    def liveness(self):
        """구동계가 명령을 받아 갈 상태인가. 판정할 수단이 없으면 None.

        `domain.task.base_liveness.LivenessVerdict`를 돌려준다. 추상이 아닌
        이유: 이것을 알 수 있는 것은 ROS 그래프를 볼 수 있는 실기 어댑터뿐이고,
        테스트 더블은 알 수도 없고 알 필요도 없다. None은 "고장 아님"이 아니라
        **"모른다"**이며, 미션은 그 둘을 구분해서 다룬다.

        ⚠️ 이 판정은 `cmd_vel` 아래가 살아 있는지만 본다. **바퀴가 실제로
        도는지는 아니다** — 그것을 아는 수단이 이 차량에 없다. 자세한 근거는
        `base_liveness` 모듈 docstring 참고(2026-08-28 정지 실패 사고)."""
        return None
    @abstractmethod
    def apply_velocity(self, linear_x: float, linear_y: float,
                       angular_z: float) -> None:
        """받은 속도를 그대로 낸다 (m/s, m/s, rad/s).

        **판단하지 않는다.** 속도 크기 제한과 명령 유효성 검사는 이미
        `domain/task/motion.py`가 끝냈고, 여기 도달한 값은 낼 수 있는 값이다.
        어댑터가 다시 자르면 한계가 두 곳에 생겨 어느 쪽이 실효인지 알 수
        없게 된다.

        실패해도 돌려줄 값이 없다 — cmd_vel은 fire-and-forget이다."""

    @abstractmethod
    def creep_forward(self, distance_m: float) -> bool:
        """정지 상태에서 정확히 이만큼 앞으로 밀고 멈춘다. 실패하면 False.

        GRASP 전용이다 — 팔이 바닥 자세로 내려가 그리퍼가 열린 상태에서,
        물체를 벌어진 턱 사이로 **밀어 넣기 위한** 이동이다. 그래서 Host의
        속도 명령이 아니라 Pi가 스스로 낸다: 이 순간의 정지 조건은 좌표가
        아니라 "턱 사이에 들어왔는가"라 오버헤드로는 볼 수 없다.

        ⚠️ 데드밴드 주의 — 0.05 m/s 아래로는 바퀴가 안 도는데 /odom_raw는
        움직였다고 보고한다(2026-08-24 실기). 구현은 짧은 버스트로 나눠야
        한다."""

    @abstractmethod
    def stop(self) -> None:
        """즉시 정지 (cmd_vel 0).

        E-STOP 경로다 — **응답을 기다리지 않는다.** 실패해도 돌려줄 값이
        없으므로 로그만 남긴다."""
