"""자연어 지시(타이핑/음성 텍스트)를 Claude API 로 라벨 하나로 해석하는 비동기 워커.

geti_detector.GetiWorker 와 같은 패턴이다 — Claude API 호출은 (수백ms~수초)
시간이 걸리는데, 그동안 메인 루프가 멈추면 그 사이 차량한테 명령이 하나도
안 나가서 워치독이 걸릴 수 있다. 그래서 API 호출은 백그라운드 스레드에서
하고, 메인 루프는 poll_result() 로 논블로킹으로만 확인한다.

지시 문장이 라벨을 직접 말하지 않아도("자유롭게 움직이는 기물 잡아줘")
Claude 가 체스 규칙/사물 생김새 같은 상식으로 추론해서, 지금 화면에 실제로
보이는 라벨(visible_labels) 중 하나를 고르게 한다 — 안 보이는 라벨을
후보로 주면 화면에 없는 걸 골라버릴 수 있어서, 매 요청마다 그 순간
보이는 라벨 목록을 같이 넘긴다.

라벨뿐 아니라 "의도"(intent)도 같이 판단한다 — "퀸 가져와" 처럼 사용자에게
직접 가져다달라는 뜻이면 "fetch", "정리해"/"치워" 처럼 원래 정해진 상자에
넣으라는 뜻(또는 딱히 구분이 안 되는 기본 지시)이면 "organize" 다. 이 값에
따라 run_mission.py/mission.py 가 목적지를 mission_config.DELIVER_HERE_XY
(사용자 앞 고정 좌표)로 할지, 기존 라벨별 상자로 할지 정한다.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

import anthropic

MODEL = "claude-opus-5"

_SYSTEM = (
    "너는 탑뷰 카메라로 보드게임 기물/장난감을 집어 나르는 로봇의 지시 해석기다. "
    "사용자는 라벨을 직접 말하지 않고 간접적으로 표현할 수 있다(예: "
    "'자유롭게 움직이는 기물' -> 체스에서 퀸이 가장 자유롭게 움직인다, "
    "'동그랗게 생긴 거' -> 축구공). 체스 기물 이동 규칙이나 사물의 생김새 같은 "
    "일반 상식으로 추론해서, 이번 요청에 같이 주어지는 '지금 보이는 라벨' 목록 "
    "중에서만 하나를 정확히 골라라. 그 목록에 없는 것을 지시하거나, 여러 개가 "
    "똑같이 그럴듯해서 확신할 수 없으면 matched=false 로 답하고 이유를 적어라.\n\n"
    "라벨과 별개로 intent(의도)도 판단해라:\n"
    "- \"fetch\": 사용자에게 직접 가져다달라는 뜻. 예: '가져와', '가져다줘', "
    "'가지고 와', '나한테 줘', '이리 줘'\n"
    "- \"organize\": 원래 정해진 상자(체스말->체스 상자, 나머지->장난감 상자)에 "
    "넣으라는 뜻. 예: '정리해', '치워', '상자에 넣어줘'. **어느 쪽인지 애매하거나 "
    "그냥 라벨만 말했으면(예: '퀸') 기본값으로 organize 를 선택해라** — 안전한 "
    "쪽(원래 정해진 상자)이 기본이어야 한다."
)

_NO_MATCH = "_no_match"   # target_label enum 에 null 대신 쓰는 값(엄격한 스키마 검증 통과용)


@dataclass
class InstructionResult:
    matched: bool
    target_label: Optional[str]
    reasoning: str
    intent: str = "organize"   # "fetch"(사용자에게) | "organize"(기존 상자로, 기본값)
    error: Optional[str] = None


class InstructionResolver:
    """submit() 은 즉시 리턴(백그라운드 스레드에서 API 호출), poll_result()
    는 논블로킹으로 결과를 확인한다. 결과는 poll_result() 를 한 번 부르면
    소비되어 다시 None 이 된다(같은 결과를 두 번 처리하면 안 되는 일회성
    이벤트라 — vehicle_link.ConsoleVehicleLink.poll_status() 와 같은 패턴)."""

    def __init__(self) -> None:
        # anthropic.Anthropic() 은 ANTHROPIC_API_KEY 환경변수(또는 `ant auth
        # login` 프로필)를 자동으로 찾는다 — 여기서 키를 직접 안 넘긴다.
        self._client = anthropic.Anthropic()
        self._lock = threading.Lock()
        self._result: Optional[InstructionResult] = None
        self._busy = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def submit(self, text: str, visible_labels: list[str]) -> None:
        """이전 요청이 아직 처리 중이면 무시한다(전송 버튼 연타 방지)."""
        with self._lock:
            if self._busy:
                return
            self._busy = True
            self._result = None
        threading.Thread(target=self._run, args=(text, visible_labels), daemon=True).start()

    def _run(self, text: str, visible_labels: list[str]) -> None:
        try:
            result = self._call_api(text, visible_labels)
        except Exception as exc:
            # API 가 실패해도(네트워크 끊김, 키 오류 등) 메인 루프는 절대
            # 죽으면 안 된다 — 에러를 결과로 감싸서 poll_result() 로 넘긴다.
            result = InstructionResult(False, None, "", error=str(exc))
        with self._lock:
            self._result = result
            self._busy = False

    def _call_api(self, text: str, visible_labels: list[str]) -> InstructionResult:
        if not visible_labels:
            return InstructionResult(False, None, "지금 화면에 보이는 기물이 없음")

        tool = {
            "name": "resolve_piece",
            "description": "지시 문장이 어떤 기물을 말하는지 판단해서 알려준다.",
            "input_schema": {
                "type": "object",
                "properties": {
                    "matched": {
                        "type": "boolean",
                        "description": "지금 보이는 라벨 중 하나로 확실히 판단했으면 true",
                    },
                    "target_label": {
                        "type": "string",
                        "enum": [*visible_labels, _NO_MATCH],
                        "description": f"matched=false 면 \"{_NO_MATCH}\"",
                    },
                    "reasoning": {
                        "type": "string",
                        "description": "왜 이렇게 판단했는지 한국어 한 문장",
                    },
                    "intent": {
                        "type": "string",
                        "enum": ["fetch", "organize"],
                        "description": "fetch=사용자에게 가져다줌, organize=기존 상자로(기본값)",
                    },
                },
                "required": ["matched", "target_label", "reasoning", "intent"],
                "additionalProperties": False,
            },
            "strict": True,
        }

        response = self._client.messages.create(
            model=MODEL,
            max_tokens=1024,
            system=_SYSTEM,
            output_config={"effort": "medium"},
            tools=[tool],
            tool_choice={"type": "tool", "name": "resolve_piece"},
            messages=[{
                "role": "user",
                "content": f"지금 보이는 라벨: {', '.join(visible_labels)}\n지시: {text}",
            }],
        )

        for block in response.content:
            if block.type == "tool_use":
                data = block.input
                label = data.get("target_label")
                matched = bool(data.get("matched")) and label != _NO_MATCH
                return InstructionResult(
                    matched=matched,
                    target_label=label if matched else None,
                    reasoning=data.get("reasoning", ""),
                    intent=data.get("intent", "organize"),
                )
        return InstructionResult(False, None, "모델이 예상한 형식으로 답하지 않음")

    def poll_result(self) -> Optional[InstructionResult]:
        with self._lock:
            result, self._result = self._result, None
            return result
