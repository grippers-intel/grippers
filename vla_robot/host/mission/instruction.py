"""자연어 지시 -> (어떤 기물을, 몇 개, 어디로) — Claude API, 백그라운드 스레드.

팀원 `hardware/host-mac/host/instruction_resolver.py` 를 옮겨 왔다. 바뀐 점:
- 대상이 **라벨 여러 개**일 수 있다("체스 말만 전부 정리해줘" = queen·knight·rook)와 개수(하나/전부).
- 출력은 구조화 출력(output_config.format, JSON 스키마)으로 받는다. 라벨은 **지금 보이는 라벨만**
  enum 으로 준다 — 화면에 없는 것을 고를 수 없게.
- 모델 `claude-opus-5-5`(effort 는 설정, 기본 low — 분류 한 번이라 깊게 생각할 일이 아니다).
  정책 거절 시 서버가 다른 모델로 다시 돌리게 `fallbacks="default"` 를 켠다(베타 헤더 필요).

API 호출은 수백 ms~수 초라 메인 루프(10 Hz, 매 사이클 Pi 에 명령)를 막으면 워치독이 걸린다.
submit() 은 바로 돌아오고, poll() 은 결과를 한 번만 내준다(같은 결과를 두 번 처리하지 않게).

키는 `ANTHROPIC_API_KEY` 환경변수(또는 `ant auth login` 프로필)에서 SDK 가 찾는다 — 파일·저장소에 두지 않는다.
"""
from __future__ import annotations

import json
import threading
from dataclasses import dataclass, field
from typing import Optional

LABEL_KO = {"rook": "룩", "soccer": "축구공", "queen": "퀸", "knight": "나이트", "star": "별", "box": "상자(작은 박스)"}
CHESS = ("queen", "knight", "rook")
NO_KEY = "Claude API 키가 없습니다 — ANTHROPIC_API_KEY 사용자 환경변수를 등록하고 run_host 를 다시 띄우세요"
NO_MATCH = "_no_match"            # enum 에 null 대신 — 엄격한 스키마 검증을 통과시키려고

SYSTEM = (
    "너는 탑뷰 카메라로 장판 위 기물을 집어 나르는 로봇의 지시 해석기다. 사용자의 한국어 지시를 읽고 "
    "어떤 기물(라벨)을, 몇 개, 어디로 옮길지 정한다.\n\n"
    "라벨 뜻: " + ", ".join(f"{k}={v}" for k, v in LABEL_KO.items()) + ". "
    f"체스 말은 {', '.join(CHESS)} 이다.\n"
    "사용자는 라벨을 직접 말하지 않을 수 있다(예: '자유롭게 움직이는 기물' -> 체스에서 가장 자유로운 퀸, "
    "'동그란 거' -> 축구공, 'L자로 움직이는 말' -> 나이트). 체스 규칙과 사물 생김새 같은 상식으로 추론해서, "
    "요청마다 함께 주는 '지금 보이는 라벨' 중에서만 고른다. 목록에 없는 것을 지시했거나, 지시가 기물과 "
    "무관하거나, 후보가 똑같이 그럴듯해서 확신할 수 없으면 matched=false 로 하고 이유를 적는다.\n\n"
    "labels: 옮길 라벨들(우선순위 순). 하나만 말했으면 하나, '체스 말 전부'처럼 묶음이면 여럿.\n"
    "quantity: 'one' = 하나만(예: '퀸 가져와', '공 하나 옮겨'), 'all' = 그 라벨들을 보이는 대로 전부"
    "(예: '전부', '다', '모두', 복수형). 애매하면 'one'.\n"
    "intent: 'fetch' = 사용자에게 직접 가져다 달라는 뜻('가져와', '가져다줘', '나한테 줘', '이리 줘'), "
    "'organize' = 상자(바구니)에 넣으라는 뜻('정리해', '치워', '넣어줘', '옮겨줘'). 애매하거나 라벨만 말했으면 "
    "'organize' — 안전한 쪽이 기본이다.\n"
    "reply: 화면에 띄울 짧은 한국어 확인 문장(예: '퀸 1개를 바구니에 넣겠습니다'). "
    "reason: 왜 그렇게 해석했는지 한 문장."
)


@dataclass
class Instruction:
    text: str
    ok: bool
    labels: tuple[str, ...] = ()
    quantity: str = "one"            # one | all
    intent: str = "organize"         # organize | fetch
    reply: str = ""
    reason: str = ""
    error: Optional[str] = None      # API·설정 오류(해석 실패와 구분)
    request_id: Optional[str] = None
    visible: tuple[str, ...] = field(default_factory=tuple)


def schema_for(visible: list[str]) -> dict:
    return {
        "type": "object",
        "properties": {
            "matched": {"type": "boolean"},
            "labels": {"type": "array", "items": {"type": "string", "enum": sorted(set(visible)) + [NO_MATCH]}},
            "quantity": {"type": "string", "enum": ["one", "all"]},
            "intent": {"type": "string", "enum": ["organize", "fetch"]},
            "reply": {"type": "string"},
            "reason": {"type": "string"},
        },
        "required": ["matched", "labels", "quantity", "intent", "reply", "reason"],
        "additionalProperties": False,
    }


def parse_result(text: str, raw: dict, visible: list[str]) -> Instruction:
    """모델 JSON -> Instruction. 스키마가 막아 주지만, 보이는 라벨 밖·중복·빈 목록은 여기서도 거른다."""
    labels = []
    for lb in raw.get("labels") or []:
        if lb in visible and lb not in labels:
            labels.append(lb)
    ok = bool(raw.get("matched")) and bool(labels)
    return Instruction(
        text=text, ok=ok, labels=tuple(labels),
        quantity="all" if raw.get("quantity") == "all" else "one",
        intent="fetch" if raw.get("intent") == "fetch" else "organize",
        reply=str(raw.get("reply") or ""), reason=str(raw.get("reason") or ""),
        visible=tuple(visible))


class InstructionResolver:
    def __init__(self, icfg) -> None:
        self.cfg = icfg
        self._lock = threading.Lock()
        self._result: Optional[Instruction] = None
        self._busy = False
        self._client = None
        self.init_error: Optional[str] = None
        try:
            import anthropic
            # 키는 SDK 가 환경변수/프로필에서 찾는다. 없으면 첫 요청에서 인증 오류가 난다.
            self._client = anthropic.Anthropic(timeout=icfg.timeout_s, max_retries=1)
        except ImportError:
            self.init_error = "anthropic 패키지가 없습니다 (pip install -r requirements.txt)"
        except Exception as exc:  # noqa: BLE001 — 설정 오류는 화면에 띄우고 계속 돈다
            self.init_error = f"Claude 클라이언트 초기화 실패: {exc}"

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def submit(self, text: str, visible_labels: list[str]) -> bool:
        """처리 중이면 무시하고 False(연타 방지)."""
        with self._lock:
            if self._busy:
                return False
            self._busy = True
        threading.Thread(target=self._run, args=(text, list(visible_labels)),
                         name="instruction", daemon=True).start()
        return True

    def poll(self) -> Optional[Instruction]:
        with self._lock:
            r, self._result = self._result, None
            return r

    def _finish(self, r: Instruction) -> None:
        with self._lock:
            self._result = r
            self._busy = False

    def _run(self, text: str, visible: list[str]) -> None:
        try:
            self._finish(self.resolve(text, visible))
        except Exception as exc:  # noqa: BLE001 — 스레드가 조용히 죽으면 화면이 "해석 중"에 멈춘다
            self._finish(Instruction(text, False, error=f"해석 중 오류: {exc}", visible=tuple(visible)))

    def resolve(self, text: str, visible: list[str]) -> Instruction:
        """블로킹 한 번. 테스트·도구에서 직접 부른다."""
        if self.init_error or self._client is None:
            return Instruction(text, False, error=self.init_error or "Claude 클라이언트 없음", visible=tuple(visible))
        if not visible:
            return Instruction(text, False, reason="작업 구역에 보이는 기물이 없습니다", visible=())
        import anthropic
        try:
            resp = self._client.beta.messages.create(
                model=self.cfg.model,
                max_tokens=4096,
                betas=["server-side-fallback-2026-07-01"],
                fallbacks="default",
                output_config={"effort": self.cfg.effort,
                               "format": {"type": "json_schema", "schema": schema_for(visible)}},
                system=SYSTEM,
                messages=[{"role": "user", "content":
                           f"지금 보이는 라벨: {', '.join(sorted(set(visible)))}\n지시: {text}"}],
            )
        except anthropic.AuthenticationError:
            return self._err(text, visible, "Claude API 키가 틀렸습니다 (ANTHROPIC_API_KEY)")
        except TypeError as exc:
            # 키·토큰·프로필이 하나도 없으면 SDK 는 요청을 만들 때 TypeError 를 낸다(1.11 실측).
            if "authentication" not in str(exc):
                raise
            return self._err(text, visible, NO_KEY)
        except anthropic.PermissionDeniedError:
            return self._err(text, visible, "API 키 권한이 없습니다")
        except anthropic.RateLimitError:
            return self._err(text, visible, "요청이 많아 잠시 막혔습니다 — 조금 뒤 다시")
        except anthropic.BadRequestError as exc:
            return self._err(text, visible, f"요청 형식 오류: {exc.message}")
        except anthropic.APIStatusError as exc:
            return self._err(text, visible, f"Claude API 오류 {exc.status_code}")
        except anthropic.APIConnectionError:
            return self._err(text, visible, "Claude API 에 연결할 수 없습니다 (인터넷 확인)")
        rid = getattr(resp, "_request_id", None)
        if resp.stop_reason == "refusal":
            return self._err(text, visible, "이 지시는 처리할 수 없습니다", rid)
        if resp.stop_reason == "max_tokens":
            return self._err(text, visible, "응답이 잘렸습니다 — 다시 말해 주세요", rid)
        body = next((b.text for b in resp.content if getattr(b, "type", "") == "text"), None)
        if body is None:
            return self._err(text, visible, "응답에 결과가 없습니다", rid)
        try:
            raw = json.loads(body)
        except json.JSONDecodeError:
            return self._err(text, visible, "응답을 읽을 수 없습니다", rid)
        r = parse_result(text, raw, visible)
        r.request_id = rid
        return r

    @staticmethod
    def _err(text, visible, msg, rid=None) -> Instruction:
        return Instruction(text, False, error=msg, request_id=rid, visible=tuple(visible))
