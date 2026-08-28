"""Geti 에서 내려받은 배포(deployment) 폴더가 제대로 놓였는지 확인한다.

Geti 플랫폼에서 export 한 zip 을 풀어 넣은 뒤, run_mission.py 를 돌리기 전에
한 번 돌려 보는 용도다. run_mission.py 는 카메라 두 대와 모델을 한꺼번에
붙잡고 시작하기 때문에, 뭔가 잘못됐을 때 원인이 모델인지 카메라인지 구분이
안 된다 — 여기서 모델만 따로 떼어 확인한다.

확인하는 것
    1. geti_sdk-deployment/deployment/ 폴더 구조 (project.json, 태스크 폴더)
    2. Deployment.from_folder() + load_inference_models() 가 되는지, 몇 초 걸리는지
    3. 실제 추론 한 번 (카메라가 있으면 카메라 프레임, 없으면 검은 프레임)
    4. 모델 라벨이 mission_config.PIECE_DEST_BOX / live_map.GLYPHS 와 맞는지

사용법
    python check_geti.py
    python check_geti.py --device GPU      # 내장 GPU 로 추론해 보기
    python check_geti.py --cam 0           # 이 카메라 프레임으로 추론 (기본: 카메라 안 씀)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

# config.py 는 aruco/ 하위폴더에 있다(팀원이 동기화하는 파일이라 건드리지 않는다).
sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg
import mission_config as mcfg

ROOT = Path(__file__).parent
DEPLOYMENT_DIR = ROOT / "geti_sdk-deployment" / "deployment"

_ok_count = 0
_fail_count = 0


def ok(msg: str) -> None:
    global _ok_count
    _ok_count += 1
    print(f"  [OK]   {msg}")


def fail(msg: str, hint: str = "") -> None:
    global _fail_count
    _fail_count += 1
    print(f"  [실패] {msg}")
    if hint:
        for line in hint.splitlines():
            print(f"         {line}")


def warn(msg: str) -> None:
    print(f"  [주의] {msg}")


def check_layout() -> list[str]:
    """폴더 구조를 확인하고 태스크 폴더 이름들을 돌려준다."""
    print("\n1. 폴더 구조")

    if not DEPLOYMENT_DIR.is_dir():
        parent = DEPLOYMENT_DIR.parent
        hint = (
            f"{DEPLOYMENT_DIR} 를 만들어야 합니다.\n"
            "Geti 에서 받은 zip 을 풀면 보통 'deployment' 폴더가 나옵니다 —\n"
            f"그 폴더를 통째로 {parent} 안에 넣으세요."
        )
        if parent.is_dir():
            hint += f"\n지금 {parent} 안에 있는 것: {[p.name for p in parent.iterdir()]}"
        fail(f"배포 폴더가 없습니다: {DEPLOYMENT_DIR}", hint)
        return []
    ok(f"배포 폴더 있음: {DEPLOYMENT_DIR}")

    project_json = DEPLOYMENT_DIR / "project.json"
    if not project_json.is_file():
        inner = DEPLOYMENT_DIR / "deployment"
        hint = ""
        if inner.is_dir():
            hint = ("'deployment/deployment' 처럼 한 겹 더 들어가 있습니다.\n"
                    "안쪽 폴더 내용을 한 단계 위로 올리세요.")
        else:
            hint = f"지금 폴더 안에 있는 것: {[p.name for p in DEPLOYMENT_DIR.iterdir()]}"
        fail("project.json 이 없습니다", hint)
        return []
    ok("project.json 있음")

    try:
        project = json.loads(project_json.read_text(encoding="utf-8"))
    except Exception as exc:
        fail(f"project.json 을 읽을 수 없습니다: {exc}")
        return []

    # 학습 가능한 태스크 이름이 곧 하위 폴더 이름이다 (geti_sdk 가 그렇게 찾는다).
    tasks = [t.get("title") for t in project.get("pipeline", {}).get("tasks", [])
             if t.get("task_type") not in (None, "dataset")]
    found = []
    for title in tasks:
        if title is None:
            continue
        folder = DEPLOYMENT_DIR / title
        if not folder.is_dir():
            continue
        missing = [n for n in ("model.json", "model", "python")
                   if not (folder / n).exists()]
        if missing:
            fail(f"태스크 폴더 '{title}' 에 빠진 것: {missing}")
        else:
            ok(f"태스크 폴더 '{title}' 정상 (model.json · model/ · python/)")
            found.append(title)

    if not found:
        fail("쓸 수 있는 태스크 폴더를 못 찾았습니다",
             f"project.json 의 태스크: {tasks}\n"
             f"폴더 안에 있는 것: {[p.name for p in DEPLOYMENT_DIR.iterdir()]}")
    return found


def check_load(device: str):
    """모델을 실제로 불러온다. 성공하면 Deployment 를 돌려준다."""
    print(f"\n2. 모델 적재 ({device})")
    try:
        import geti_detector
    except Exception as exc:
        fail(f"geti_detector 를 import 할 수 없습니다: {exc}")
        return None

    t0 = time.perf_counter()
    try:
        deployment = geti_detector.load_deployment(device=device)
    except Exception as exc:
        fail(f"{type(exc).__name__}: {exc}",
             "device 를 바꿔 보거나(--device CPU), 폴더 구조를 다시 확인하세요.")
        return None
    ok(f"적재 성공 — {time.perf_counter() - t0:.1f} 초")
    return deployment


def check_infer(deployment, cam_index: int | None):
    """추론을 한 번 돌려 시간과 검출 결과를 본다."""
    print("\n3. 추론")
    import cv2

    frame = None
    if cam_index is not None:
        cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
        if cap.isOpened():
            for _ in range(10):          # 자동노출 안정화
                got, f = cap.read()
            if got and f is not None:
                frame = f
                ok(f"카메라 {cam_index} 프레임 확보 ({f.shape[1]}x{f.shape[0]}, "
                   f"평균밝기 {f.mean():.1f})")
            else:
                warn(f"카메라 {cam_index} 를 열었지만 프레임을 못 읽었습니다")
        else:
            warn(f"카메라 {cam_index} 를 열 수 없습니다")
        cap.release()

    if frame is None:
        frame = np.zeros((cfg.IMG_H, cfg.IMG_W, 3), dtype=np.uint8)
        warn("검은 프레임으로 추론합니다 — 속도만 재고, 검출 결과는 의미 없습니다")

    times = []
    prediction = None
    for i in range(3):
        t0 = time.perf_counter()
        try:
            prediction = deployment.infer(frame)
        except Exception as exc:
            fail(f"추론 실패 — {type(exc).__name__}: {exc}")
            return
        times.append(time.perf_counter() - t0)
    ok(f"추론 {len(times)}회 — 첫 회 {times[0]:.2f}초, 이후 평균 "
       f"{sum(times[1:]) / max(len(times) - 1, 1):.2f}초")

    annotations = getattr(prediction, "annotations", []) or []
    if annotations:
        seen = {}
        for ann in annotations:
            for lab in getattr(ann, "labels", []):
                name = getattr(lab, "name", "?")
                prob = getattr(lab, "probability", 0.0) or 0.0
                seen[name] = max(seen.get(name, 0.0), prob)
        ok(f"검출 {len(annotations)}개 — " +
           ", ".join(f"{k}({v:.2f})" for k, v in sorted(seen.items())))
    else:
        warn("검출 0개 (검은 프레임이었다면 정상입니다)")


def check_labels(deployment) -> None:
    """모델 라벨이 미션 설정과 맞는지 본다."""
    print("\n4. 라벨 대조")
    try:
        from live_map import GLYPHS, KNOWN_LABELS
    except Exception as exc:
        fail(f"live_map 을 import 할 수 없습니다: {exc}")
        return

    names = set()
    for task in deployment.project.get_trainable_tasks():
        for lab in getattr(task, "labels", []) or []:
            name = getattr(lab, "name", None)
            # Geti 는 배경/빈 라벨을 함께 내려주는데 미션과 무관하다.
            if name and not getattr(lab, "is_empty", False):
                names.add(name)

    if not names:
        warn("모델에서 라벨 목록을 못 읽었습니다 — 건너뜁니다")
        return
    ok(f"모델 라벨 {len(names)}종: {sorted(names)}")

    unmapped = sorted(names - set(mcfg.PIECE_DEST_BOX))
    if unmapped:
        fail(f"목적지 상자가 정해지지 않은 라벨: {unmapped}",
             "mission_config.PIECE_DEST_BOX 에 추가하세요 — 없으면 mission.py 가\n"
             "그 기물을 건너뛰고 다음 후보를 기다립니다.")
    else:
        ok("모든 라벨에 목적지 상자가 지정돼 있습니다")

    no_icon = sorted(names - set(GLYPHS) - {"box"} - set(KNOWN_LABELS))
    if no_icon:
        warn(f"LiveMap 전용 아이콘이 없는 라벨(회색 원으로 그려짐): {no_icon}")

    missing = sorted(set(mcfg.PIECE_DEST_BOX) - names)
    if missing:
        warn(f"설정에는 있는데 모델에는 없는 라벨: {missing} (모델이 바뀌었다면 정상)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Geti 배포 폴더 점검")
    ap.add_argument("--device", default="CPU", help="추론 장치 (기본 CPU)")
    ap.add_argument("--cam", type=int, default=None,
                    help="이 인덱스의 카메라 프레임으로 추론해 본다 (기본: 카메라 안 씀)")
    args = ap.parse_args()

    print(f"배포 폴더: {DEPLOYMENT_DIR}")

    if check_layout():
        deployment = check_load(args.device)
        if deployment is not None:
            check_infer(deployment, args.cam)
            check_labels(deployment)

    print(f"\n{'=' * 52}")
    if _fail_count:
        print(f"실패 {_fail_count}건 — 위 안내를 따라 고친 뒤 다시 돌리세요.")
        return 1
    print(f"통과 {_ok_count}건, 실패 0건 — run_mission.py 를 돌릴 수 있습니다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
