# _legacy — 실행되지 않던 최상위 사본

여기 있는 파일들은 `grippers_topview/` 최상위에 있던 `config.py` · `localizer.py` ·
`selftest.py` · `place_markers.py` · `calibrate_camera.py` 의 사본이다.
**삭제하지 않고 옮겨 둔 이유는 실측 기록이 남아 있기 때문**이고, 코드로서는
이미 죽어 있었다.

## 왜 죽어 있었나

미션 스크립트들은 전부 맨 위에서 이렇게 한다:

```python
sys.path.insert(0, str(Path(__file__).parent / "aruco"))   # aruco/ 를 맨 앞에
import config as cfg                                        # → aruco/config.py
from localizer import Camera, Pose, ...                     # → aruco/localizer.py
```

`aruco/` 가 `sys.path` 맨 앞에 붙으므로 `import config` 는 **언제나
`aruco/config.py`** 로 간다. 최상위 사본은 한 번도 로드되지 않았다.
`__pycache__` 가 이를 뒷받침한다 — 실제 실행 파이썬(3.11)의
`config.cpython-311.pyc` / `localizer.cpython-311.pyc` 는 `aruco/` 안에만
있었고 최상위에는 없었다.

## 두 사본의 차이 (2026-08-27 확인)

최상위 사본은 **캘리브레이션을 돌리기 전에 실측값을 적어 둔 기록본**이다.
`calib/cam0.npz`·`cam1.npz` 가 생긴 8/26 13:48~14:17 보다 먼저 쓰였다(12:57).

| 상수 | `aruco/` (정본) | `_legacy/` | 처리 |
|---|---|---|---|
| `ROBOT_MARKER_HEIGHT` | 0.200 → **0.270** | 0.270 | ✅ 정본에 반영함 |
| `CALIB_DIR` | `"calib"` → **절대경로** | 절대경로 | ✅ 정본에 반영함 |
| `IMG_W, IMG_H` | **1280×720** | 1920×1080 | ❌ 정본이 720p. 마커 크기·오차 검증이 이 해상도 기준 |
| `MAX_EXTRINSIC_REPROJ_PX` | **3.0** | 4.5 | ❌ 4.5 는 1080p 용(3.0×1.5). 720p 에는 3.0 |
| `EXTRINSIC_RELOCK_PX` | **4.0** | 6.0 | ❌ 위와 세트 |
| `HFOV_DEG` | 70.4 | 68.5 | ❌ calib npz 가 있으면 안 쓰이는 대체값 |
| `WORKSPACE_Y` | (0.400, **1.400**) | (0.400, 1.410) | ❌ 1cm. 표시·검증용 |
| `BOX_APPROACH_FROM_ENTRANCE` | 없음 | 0.144 | ❌ `_legacy/localizer.box_approach_target()` 전용인데 그 함수를 부르는 곳이 없다. 같은 일을 `mission._box_front_xy()` 가 `BOX_L/2 + BOX_APPROACH_MARGIN_M` 로 이미 한다 |
| `ROBOT_MARKER_TO_FRONT` | 없음 | 0.099 | ❌ 어디에서도 참조하지 않는다 |

## 지워도 되나

`ROBOT_MARKER_HEIGHT` 와 `CALIB_DIR` 은 정본으로 옮겼고, 나머지는 위 표대로
정본 쪽이 맞다. **실측 기록을 다른 곳에 남겼다면 이 폴더는 지워도 된다.**
지우기 전에 1080p 로 올릴 계획이 있는지만 확인할 것 — 그때는 이 파일의
해상도 3종 세트(`IMG_W/H`, `MAX_EXTRINSIC_REPROJ_PX`, `EXTRINSIC_RELOCK_PX`)가
그대로 참고자료가 된다.
