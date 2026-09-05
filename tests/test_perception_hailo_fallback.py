"""perception_node — Hailo 강제 비활성화가 되살아나지 않는지, 되살린 뒤
Hailo 로드 실패 시 CPU YOLO로 안전하게 넘어가는지 (2026-09-06, 사용자 지시).

## 배경

2026-08-22~2026-09-06 사이 `_HAILO_AVAILABLE = False`로 하드코딩 강제
비활성화돼 있었다 — AI HAT+2가 부팅마다 죽는 증상(재현 확인, 온보드 DDR
손상으로 추정)이 있었기 때문이다. 2026-09-06 저녁 Hailo 하드웨어 점검
(vcgencmd/lspci/hailortcli/dmesg)이 전부 깨끗하게 나와 사용자 지시로
되살렸다.

⚠️ 그 점검은 PCIe 링크·펌웨어 부팅까지만 확인한 것이지 실제 추론까지
검증한 게 아니다 — DDR 손상이 진짜였다면 가벼운 identify 호출이 아니라
실제 모델 추론 중에야 다시 죽을 수 있다. 그래서 되살리면서 동시에
`_load_hailo_model()`이 실패하면 CPU YOLO로 넘어가는 폴백을 추가했다 —
이 파일은 그 두 가지(강제 비활성화가 없는지 / 폴백이 있는지)를 정적으로
검증한다.

`perception_node.py`는 rclpy를 무조건 import해서 이 스위트에서 직접
import할 수 없다(test_constant_copies.py와 같은 사정) — 소스를 AST로
읽는다."""

import ast
import pathlib

REAL_PATH = (pathlib.Path(__file__).resolve().parent.parent / "ros2_ws" / "src"
             / "grippers_perception" / "grippers_perception" / "perception_node.py")


def _parse():
    return ast.parse(REAL_PATH.read_text(encoding="utf-8"), filename=str(REAL_PATH))


def _called_name(node):
    func = node.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def test_HAILO_AVAILABLE_강제_비활성화가_모듈_최상위에_없다():
    """try/except 안의 `_HAILO_AVAILABLE = True/False`(import 성공 여부에 따른
    정상 대입)는 허용한다 — 그 바깥, 모듈 최상위에 있는 추가 `= False`
    대입만 금지한다. 있으면 하드웨어가 멀쩡해도 항상 CPU 경로로 접힌다."""
    tree = _parse()
    offenders = []
    for stmt in tree.body:
        if isinstance(stmt, ast.Try):
            continue  # import try/except 안의 정상 대입은 검사 대상이 아니다
        for node in ast.walk(stmt):
            if not isinstance(node, ast.Assign):
                continue
            targets = {t.id for t in node.targets if isinstance(t, ast.Name)}
            if "_HAILO_AVAILABLE" not in targets:
                continue
            if isinstance(node.value, ast.Constant) and node.value.value is False:
                offenders.append(node.lineno)
    assert not offenders, (
        f"모듈 최상위에 _HAILO_AVAILABLE = False 강제 대입이 남아 있다 (줄 {offenders}) — "
        "하드웨어가 정상이어도 Hailo 경로를 영원히 안 탄다")


def test_hailo_로드_실패하면_CPU_YOLO로_폴백한다():
    """__init__에서 `_load_cpu_yolo_model()` 호출이 (1) `_HAILO_AVAILABLE`이
    아닐 때의 elif 분기, (2) Hailo를 시도했다가 실패했을 때의 폴백 분기,
    이렇게 최소 두 곳에 있어야 한다 — 하나뿐이면 Hailo 로드 실패 시
    scan_floor가 백엔드 없이 완전히 죽는다."""
    tree = _parse()
    init_fn = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    calls = [n for n in ast.walk(init_fn) if isinstance(n, ast.Call)]
    cpu_yolo_calls = [c for c in calls if _called_name(c) == "_load_cpu_yolo_model"]
    assert len(cpu_yolo_calls) >= 2, (
        "__init__에서 _load_cpu_yolo_model() 호출이 elif 분기 하나뿐이다 — "
        "Hailo 로드 실패 시 CPU YOLO로 넘어가는 폴백이 없다")
