"""시연 UI 입력창 -> Claude 해석(InstructionResolver) -> FSM 지시(Order). 화면이 읽을 상태도 여기 둔다.

phase:  idle -> interpreting -> (accepted | failed) -> idle
- accepted: fsm.set_order() 를 불렀다. 화면은 실행/완료로 간다.
- failed:   카드가 뜬다 — 해석 실패(E-402, 이유 + 보이는 기물에서 고르기) · 기물 없음(E-201) · API 오류.
            카드 버튼으로 닫거나(dismiss) 라벨을 직접 고른다(pick_label).
"""
from __future__ import annotations

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
        self.failure: Optional[str] = None      # "unparseable" | "error" | "empty"
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
    def update(self, fsm) -> None:
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
        fsm.set_order(order)
        self.phase, self.failure = "accepted", None
        if order.intent == "fetch":
            self.note = ("가져오기는 손 전달이 붙으면 손으로 — 지금은 바구니에 넣습니다", "FETCH", "caution")

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
