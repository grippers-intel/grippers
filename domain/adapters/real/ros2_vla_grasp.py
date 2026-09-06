"""Ros2VlaGrasp — mission_orchestrator 가 쓰는 VLA 파지 포트 구현.

`vla_inference` 노드에 액션으로 말을 건다. **정책은 이 프로세스에 없다** —
미션 노드가 torch 를 부르지 않게 하려는 것이다. classic 백엔드만 쓸 때
정책 무게를 하나도 지지 않아야 하고, 정책이 죽어도 미션 FSM 은 살아 있어야
한다.

⚠️ 타임아웃이 길다. 청크 하나가 100스텝 x 30fps = 3.33초이고, 파지 한 번에
여러 청크가 든다. 학습 회차 길이가 평균 570프레임(19초)이었으므로 그 두 배쯤을
기본으로 잡는다. 짧게 잡으면 팔은 제대로 움직이는데 호출자만 실패로 받는다 —
2026-08-28 에 set_gripper 와 offset_base_yaw 가 정확히 그렇게 잘못 보고됐다.
"""

from grippers_interfaces.action import RunVlaGrasp
from rclpy.action import ActionClient

from domain.adapters.real._ros_call import call_action

#: 파지 한 번의 **바깥쪽** 대기 상한(초).
#:
#: 진짜 예산은 노드가 계산한다(grippers_vla.grasp_budget) — 청크 길이가
#: 정책마다 다르기 때문이다. 여기 값은 "그보다 넉넉한 바깥 벽"이기만 하면
#: 된다. 짧으면 아래 주석의 사고가 난다: **노드는 아직 돌고 있는데 호출자가
#: 먼저 포기**해서, 팔이 움직이는 채로 실패가 보고된다.
#:
#: ⚠️ 2026-09-06 에 45 -> 75 로 올렸다. 45 는 ACT 기준(학습 회차 평균 19초의
#: 두 배 + 여유)이었는데, DP 는 청크가 1.07초라 노드 예산이 32청크 x
#: (1.07 + 0.5) = 50초까지 간다. 바깥 벽이 55초(45+10)면 여유가 3초뿐이라,
#: 한 청크만 느려도 호출자가 먼저 포기한다.
#:
#: 올려도 ACT 는 안 느려진다 — 이건 상한이지 대기 시간이 아니다. ACT 는
#: 여전히 35초 안팎에 끝나고 그 시점에 결과를 돌려준다.
VLA_GRASP_TIMEOUT_SEC = 75.0


class Ros2VlaGrasp:
    """포트 프로토콜: `run_grasp(label) -> bool`."""

    def __init__(self, node):
        self._node = node
        self._client = ActionClient(node, RunVlaGrasp, "vla_inference/run_grasp")

    def run_grasp(self, label: str) -> bool:
        """정책 루프를 끝까지 돌렸으면 True.

        ⚠️ True 가 "물체를 집었다"는 뜻이 **아니다.** 진짜 판정은 호출부가
        classic 과 같은 두 신호(부하 + 뎁스)로 한다 — RunVlaGrasp.action 의
        같은 경고 참고. 여기서 성공을 판정하면 정책이 자기 실패를 스스로
        판정하는 꼴이 된다.
        """
        goal = RunVlaGrasp.Goal(label=label, timeout_s=float(VLA_GRASP_TIMEOUT_SEC))
        result = call_action(
            self._node,
            self._client,
            goal,
            label="run_vla_grasp",
            # 수락은 즉시여야 하지만(기본값 유지), 결과는 파지가 끝날 때까지
            # 걸린다. 노드 쪽 상한보다 넉넉히 잡아야 "노드는 실패로 접었는데
            # 호출자는 이미 포기한" 상태가 안 생긴다.
            result_timeout_sec=VLA_GRASP_TIMEOUT_SEC + 10.0,
        )
        return result is not None and result.ok
