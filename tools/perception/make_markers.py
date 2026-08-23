#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""인쇄용 ArUco 마커 생성.

**여백(quiet zone)이 검출의 절반이다.** 마커 테두리에 흰 여백이 없으면 검출기가
바깥 사각형을 못 잡는다. 마커 한 변의 최소 15% 를 흰색으로 두른다.

크기는 검출 거리를 정한다. 640px 폭 카메라에서 대략 —
  100mm 마커 → 2m 에서도 안정적
   60mm 마커 → 1m 안쪽
바구니는 멀리서부터 찾아야 하므로 크게 뽑는 편이 낫다.
"""
import argparse
import os

import cv2
import numpy as np

DPI = 300
MM_PER_INCH = 25.4


def mm2px(mm: float) -> int:
    return int(round(mm / MM_PER_INCH * DPI))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3, 4, 5])
    ap.add_argument("--mm", type=float, default=100.0, help="마커 한 변(mm)")
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--out", default="/grippers/markers")
    args = ap.parse_args()

    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))
    side = mm2px(args.mm)
    quiet = int(round(side * 0.15))
    os.makedirs(args.out, exist_ok=True)

    for i in args.ids:
        img = cv2.aruco.generateImageMarker(d, i, side)
        canvas = np.full((side + 2 * quiet, side + 2 * quiet), 255, np.uint8)
        canvas[quiet:quiet + side, quiet:quiet + side] = img
        # 아래에 사람이 읽을 라벨을 붙인다 — 붙이고 나서 어느 ID 인지 알아야 한다.
        label = np.full((90, canvas.shape[1]), 255, np.uint8)
        cv2.putText(label, f"ID {i}  /  {args.dict}  /  {args.mm:.0f}mm",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.1, 0, 2, cv2.LINE_AA)
        sheet = np.vstack([canvas, label])
        path = os.path.join(args.out, f"aruco_{args.dict}_{i}_{args.mm:.0f}mm.png")
        cv2.imwrite(path, sheet)
        print(f"  {path}  ({sheet.shape[1]}×{sheet.shape[0]}px @ {DPI}dpi)")

    print(f"\n{len(args.ids)}장 생성. 인쇄할 때 '실제 크기 100%' 로 두세요 — "
          f"'페이지에 맞춤' 을 쓰면 크기가 틀어집니다.")


if __name__ == "__main__":
    main()
