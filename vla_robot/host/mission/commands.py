"""시연 UI 입력창 -> Claude 해석(InstructionResolver) -> FSM 지시(Order). 화면이 읽을 상태도 여기 둔다.

phase:  idle -> interpreting -> (accepted | failed) -> idle
- accepted: fsm.set_order() 를 불렀다. 화면은 실행/완료로 간다.
- failed:   카드가 뜬다 — 해석 실패(E-402, 이유 + 보이는 기물에서 고르기) · 기물 없음(E-201) · API 오류.
            카드 버튼으로 닫거나(dismiss) 라벨을 직접 고른다(pick_label).
"""
from __future__ import annotations

import time
from typing import Optional

from mission.host_fsm import Order
from mission.instruction import Instruction, InstructionResolver

KO = {"rook": "룩", "soccer": "공", "queen": "퀸", "knight": "나이트", "star": "별", "box": "상자"}


def visible_labels(cfg, pmap) -> list[str]:
    a = cfg.arena
    return sorted({lb for lb, pts in pmap.items() for x, y in pts
                   if a.workspace_x[0] <= x <= a.workspace_x[1] and a.workspace_y[0] <= y <= a.workspace_y[1]})


class CommandDesk:
    def __init__(self, cfg, resolver=None) -> None:
        self.cfg = cfg
        self.resolver = resolver if resolver is not None else InstructionResolver(cfg.instruction)
        self.phase = "idle"
        self.text = ""
        self.result: Optional[Instruction] = None
        self.failure: Optional[str] = None      # "unparseable" | "error" | "empty" | "nohand"
        self.hands: list = []                   # 매 사이클 update() 가 받는다 — "가져와"는 손이 보여야 접수
        self._pending: Optional[Order] = None   # 손이 없어 멈춘 "가져와" 지시(카드에서 바구니로 바꿀 수 있다)
        self._want_since: Optional[float] = None   # 손 검출을 켠 시각 — 켠 직후에는 손이 아직 확인 전이다
        self._hand_wait_until: Optional[float] = None
        self.visible: list[str] = []
        self.note: Optional[tuple[str, str, str]] = None   # (text, code, tone) — 한 번 띄울 알림

    # -- 입력 ---------------------------------------------------------------
    def submit(self, text: str, pmap) -> None:
        text = text.strip()
        if not text:
            return
        if self.phase == "interpreting":
            self.note = ("앞 명령을 해석하는 중입니다 — 잠시만요", "CMD", "caution")
            return
        self.text, self.result, self.failure = text, None, None
        self.visible = visible_labels(self.cfg, pmap)
        if not self.visible:
            self.phase, self.failure = "failed", "empty"
            return
        if self.resolver.submit(text, self.visible):
            self.phase = "interpreting"

    def dismiss(self) -> None:
        self.phase, self.failure = "idle", None

    def pick_label(self, label: str, fsm) -> None:
        """E-210 카드에서 사람이 직접 고른 라벨 — 해석 없이 바로 지시로."""
        if label not in KO and label not in self.visible:
            return
        intent = self.result.intent if self.result else "organize"
        self._accept(fsm, Order((label,), "one", intent, self.text or label))

    # -- 매 사이클 ----------------------------------------------------------
    def hands_wanted(self, fsm) -> bool:
        """손 검출이 필요한가 — 명령 해석 중("가져와"일 수 있다) · 손을 기다리는 중 · 손에 건네는 지시가 도는 중.
        그 밖에는 끈다(10-08 사용자: "가져와" 때만 실시간으로 본다)."""
        order = getattr(fsm, "order", None)
        return (self.phase in ("interpreting", "hand_wait") or self._pending is not None
                or (order is not None and order.intent == "fetch"))

    def update(self, fsm, hands=()) -> None:
        self.hands = list(hands)
        now = time.monotonic()
        if self.hands_wanted(fsm):
            self._want_since = self._want_since or now
        else:
            self._want_since = None
        if self.phase == "hand_wait" and self._pending is not None:
            if self.hands:
                order, self._pending = self._pending, None
                self._accept(fsm, order)
            elif now >= (self._hand_wait_until or 0.0):
                self.phase, self.failure = "failed", "nohand"
            return
        r = self.resolver.poll()
        if r is None:
            return
        self.result = r
        if r.ok:
            self._accept(fsm, Order(r.labels, r.quantity, r.intent, r.text))
        else:
            # 해석 실패(보이지 않는 기물·애매함)는 이유와 함께 보이는 기물에서 고르게 한다. API 오류는 따로.
            self.phase = "failed"
            self.failure = "error" if r.error else "unparseable"

    def _accept(self, fsm, order: Order) -> None:
        if order.intent == "fetch" and not self.hands:
            # 손이 안 보이면 시작하지 않는다(2026-10-01 결정). 카드에서 바구니로 바꾸거나 손을 내밀고 다시.
            # 손 검출은 해석을 시작할 때 켜진다 — 확인(hands.confirm_s)까지 HAND_WARMUP_S 는 기다려 본다.
            self._pending = order
            ready_at = (self._want_since or time.monotonic()) + self.HAND_WARMUP_S
            if time.monotonic() < ready_at:
                self.phase, self.failure, self._hand_wait_until = "hand_wait", None, ready_at
                return
            self.phase, self.failure = "failed", "nohand"
            return
        fsm.set_order(order)
        self.phase, self.failure = "accepted", None

    def to_basket(self, fsm) -> None:
        """손이 없을 때 카드의 "바구니에 넣기" — 같은 지시를 정리로 바꿔 접수."""
        order = self._pending
        if order is not None:
            self._pending = None
            order.intent = "organize"
            fsm.set_order(order)
            self.phase, self.failure = "accepted", None

    #: 손 검출을 켠 뒤 "손 없음"이라 하기 전에 기다리는 시간 — 확인 0.6 s + 카메라 두 대 추론·지연 여유.
    HAND_WARMUP_S = 2.0

    # -- 화면 ---------------------------------------------------------------
    def card(self) -> Optional[dict]:
        if self.phase != "failed":
            return None
        r = self.result
        if self.failure == "empty":
            return {"code": "E-201 TARGET_NOT_FOUND", "next": "→ IDLE", "tone": "caution", "icon": "!",
                    "title": "작업 구역에 기물이 없습니다",
                    "detail": "카메라에 잡히는 기물이 없습니다. 기물을 놓고 다시 말해 주세요.",
                    "rows": [], "actions": [{"id": "dismiss", "label": "확인", "primary": True}]}
        if self.failure == "nohand":
            return {"code": "E-220 NO_HAND", "next": "→ 손 대기", "tone": "caution", "icon": "!",
                    "title": "손이 보이지 않습니다",
                    "detail": "장판 앞이나 옆 가장자리에서 손바닥을 위로 펴고 1초쯤 들고 있다가 다시 말해 주세요.",
                    "rows": [], "actions": [{"id": "basket", "label": "바구니에 넣기", "primary": True},
                                            {"id": "dismiss", "label": "취소"}]}
        if self.failure == "error":
            return {"code": "E-402 UNPARSEABLE", "next": "→ IDLE", "tone": "error", "icon": "!",
                    "title": "명령을 해석할 수 없습니다", "detail": r.error if r else "",
                    "rows": [], "actions": [{"id": "dismiss", "label": "닫기", "primary": True}]}
        rows = [{"label": KO.get(lb, lb), "meta": lb, "act": "pick_label", "id": lb} for lb in self.visible]
        return {"code": "E-402 UNPARSEABLE", "next": "→ 대상 선택 대기", "tone": "caution", "icon": "!",
                "title": "무엇을 옮길지 모르겠습니다",
                "detail": "대상이 분명하지 않아 실행하지 않습니다. 다시 말하거나 아래에서 골라 주세요.",
                "heard": {"text": self.text, "hit": None, "why": r.reason if r else ""},
                "rows": rows, "actions": [{"id": "dismiss", "label": "다시 말하기", "primary": True}]}
