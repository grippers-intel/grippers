"""탑뷰 2대에서 YOLO 학습용 이미지를 동시에 캡처한다.

사용법
    python capture_dataset.py
    python capture_dataset.py --tag s4-3ea       # 세션 이름 (권장)
    python capture_dataset.py --auto 2.0         # 2초마다 자동 촬영

세션 — 한 번 실행이 한 세션이다
    파일명에 실행 시각과 --tag 가 박히므로, 나중에 train/val 을 '세션 단위'로
    나눌 수 있다. 같은 배치를 찍은 사진끼리는 반드시 같은 세션에 둘 것
    (로봇 있음/없음을 다른 태그로 쪼개면 같은 장면이 train 과 val 로 갈라진다).

조작
    SPACE : 두 카메라를 한 번에 캡처
    a     : 자동 촬영 켜기/끄기 (--auto 로 간격 지정)
    q     : 종료

저장 구조 — 같은 순간의 두 장이 같은 파일명을 갖는다.
    dataset/cam0/20260823_0715_0001.jpg
    dataset/cam1/20260823_0715_0001.jpg

⚠️ 두 카메라는 하드웨어 동기가 아니다. 순차로 read() 하므로 수십 ms 차이가 난다.
   물체가 정지해 있으면 문제없지만, 로봇이 움직이는 장면은 그만큼 어긋난다.

찍을 때 — 매번 전부 넣지 말 것
    실제 미션은 물체를 하나씩 치우므로 4개 → 0개로 줄어든다. 4개짜리만 학습하면
    후반부에서 성능이 떨어진다. 개수를 섞고, 특히 빈 바닥을 10~15% 넣을 것
    (나뭇결 오검출 억제).

    ⚠️ 로봇은 68% 에 넣는다 — 별개 세션이 아니라 모든 세션에 걸리는 축이다.
       운용 중 탑뷰에 로봇은 '항상' 있다. 로봇이 물체를 가리고 그림자를 만들고
       옆면·바퀴가 물체로 오검출되는데, 이건 로봇이 프레임에 있어야만 배운다.
       같은 배치에서 로봇만 넣었다 뺐다 하면 한 배치로 3장이 나온다.
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2

import sys

# config.py/localizer.py 는 aruco/ 하위폴더에 있다(팀원이 동기화하는 파일이라
# 건드리지 않는다) — 그 폴더를 경로에 추가해서 기존처럼 bare import 로 쓴다.
# 예전엔 이 폴더 최상위에도 같은 이름의 사본이 있어서 이 줄 없이도 돌았지만,
# 그건 캘리브레이션 이전 실측 기록본이었고 지금은 _legacy/ 로 옮겼다.
sys.path.insert(0, str(Path(__file__).parent / "aruco"))

import config as cfg

# 카메라별 초점값 — calibrate_camera.py 와 반드시 같아야 한다.
# 지정하지 않으면 250(최근접)에 붙어 바닥이 통째로 흐려진다.
FOCUS = {0: 5, 1: 0}


def open_cam(idx: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return cap
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.IMG_H)
    # ⚠️ FOURCC 는 반드시 해상도 '뒤에'. 먼저 걸면 DSHOW 가 조용히 무시해
    #    1080p 가 5 fps 로 떨어진다 (실측: 먼저 5.0 · 나중 27.1 fps).
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    if idx in FOCUS:
        cap.set(cv2.CAP_PROP_FOCUS, FOCUS[idx])
    return cap


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", type=int, nargs="+", default=list(cfg.CAM_INDICES))
    ap.add_argument("--out", type=str, default="",
                    help="저장 폴더. 비우면 이 스크립트 옆의 dataset/. "
                         "하위에 cam0 / cam1 이 만들어진다.")
    ap.add_argument("--view-scale", type=float, default=0.4,
                    help="미리보기 창 배율. 두 창을 나란히 봐야 하므로 기본을 작게 뒀다.")
    ap.add_argument("--auto", type=float, default=0.0,
                    help="자동 촬영 간격(초). 0 이면 수동만. 실행 중 a 로 켜고 끈다.")
    ap.add_argument("--quality", type=int, default=95, help="JPEG 품질")
    ap.add_argument("--tag", type=str, default="",
                    help="세션 이름. 파일명에 들어간다 (예: s4-3ea). "
                         "train/val 을 세션 단위로 나눌 때 쓴다.")
    args = ap.parse_args()

    # 파일명에 넣을 수 없는 문자를 막는다. 한글도 뺀다 — Roboflow 업로드에서
    # 인코딩 문제를 일으킨다 (isalnum() 만 쓰면 한글이 통과하므로 isascii() 도 본다).
    tag = "".join(c for c in args.tag
                  if (c.isalnum() and c.isascii()) or c in "-_")
    if args.tag and tag != args.tag:
        print(f"태그를 '{tag}' 로 바꿨습니다 (영문·숫자·-·_ 만 가능).")

    # ⚠️ 기본값을 'dataset' 상대경로로 두면 터미널을 어디서 열었느냐에 따라
    #    엉뚱한 폴더에 쌓인다 (실측: 69쌍이 grippers/dataset 로 갔음).
    #    calib 과 같은 방식으로 스크립트 위치에 고정한다.
    out = (Path(args.out).resolve() if args.out
           else Path(__file__).resolve().parent / "dataset")
    dirs = {i: out / f"cam{i}" for i in args.cams}
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)

    caps = {i: open_cam(i) for i in args.cams}
    dead = [i for i, c in caps.items() if not c.isOpened()]
    if dead:
        print(f"카메라 {dead} 를 열 수 없습니다.")
        for c in caps.values():
            c.release()
        return 1

    time.sleep(1.0)
    for c in caps.values():          # 자동노출 안정화
        for _ in range(10):
            c.read()

    for i in args.cams:
        win = f"cam{i}"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(win, int(cfg.IMG_W * args.view_scale),
                         int(cfg.IMG_H * args.view_scale))

    print(__doc__)
    print(f"저장 위치: {out}")
    for i in args.cams:
        print(f"  cam{i}: {dirs[i]}  (초점 {FOCUS.get(i, '미지정')})")
    print()

    stamp = datetime.now().strftime("%Y%m%d_%H%M") + (f"_{tag}" if tag else "")
    print(f"이번 세션 이름: {stamp}"
          + ("" if tag else "   (--tag 로 이름을 붙일 수 있습니다)"))
    n = 0
    auto_on = False
    last_auto = 0.0
    last_msg = ""

    try:
        while True:
            # 두 장을 최대한 붙여서 읽는다 — 사이에 다른 작업을 넣지 않는다.
            frames = {}
            ok_all = True
            for i in args.cams:
                ok, f = caps[i].read()
                frames[i] = f if ok else None
                ok_all = ok_all and ok

            now = time.monotonic()
            shoot = False
            if auto_on and args.auto > 0 and now - last_auto >= args.auto:
                shoot = True
                last_auto = now

            for i in args.cams:
                f = frames[i]
                if f is None:
                    continue
                view = f.copy()
                txt = f"cam{i}  saved:{n}  SPACE=capture  a=auto({'ON' if auto_on else 'off'})  q=quit"
                cv2.putText(view, txt, (14, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 0), 5, cv2.LINE_AA)
                cv2.putText(view, txt, (14, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 255, 0) if auto_on else (255, 255, 255),
                            2, cv2.LINE_AA)
                if last_msg:
                    cv2.putText(view, last_msg, (14, 80), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 0, 0), 5, cv2.LINE_AA)
                    cv2.putText(view, last_msg, (14, 80), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (120, 220, 255), 2, cv2.LINE_AA)
                cv2.imshow(f"cam{i}", view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("a"):
                auto_on = not auto_on
                last_auto = now
                print(f"자동 촬영 {'ON' if auto_on else 'OFF'}"
                      + (f" ({args.auto}초 간격)" if auto_on and args.auto > 0 else ""))
                if auto_on and args.auto <= 0:
                    print("  ⚠️ --auto 로 간격을 지정하지 않아 동작하지 않습니다.")
            if key == ord(" "):
                shoot = True

            if shoot:
                if not ok_all:
                    print("  한쪽 프레임을 못 읽어 건너뜁니다.")
                    continue
                n += 1
                name = f"{stamp}_{n:04d}.jpg"
                for i in args.cams:
                    cv2.imwrite(str(dirs[i] / name), frames[i],
                                [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                last_msg = f"saved {name}"
                print(f"  [{n:4d}] {name}  ->  " +
                      " / ".join(f"cam{i}" for i in args.cams))
    finally:
        for c in caps.values():
            c.release()
        cv2.destroyAllWindows()

    print(f"\n총 {n} 쌍 저장 ({n * len(args.cams)} 장)")
    for i in args.cams:
        print(f"  {dirs[i]}: {len(list(dirs[i].glob('*.jpg')))} 장")
    if n:
        print("\n다음 — 개수 구성을 확인할 것 (빈 바닥 10~15% · 로봇 포함 15~20%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
