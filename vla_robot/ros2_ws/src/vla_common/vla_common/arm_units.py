"""SO-ARM101 관절값: 서보 raw <-> LeRobot 정규화 단위(= 정책 단위).

## 캘리브레이션은 하나뿐이다

기존 코드에는 "교시(TAUGHT) 오프셋"과 "LeRobot 오프셋" 두 벌이 같은 서보 EEPROM 에
번갈아 쓰였고, wrist_roll 에서 85.8도가 어긋났다. 이 프로젝트는 **LeRobot
캘리브레이션(arm_calibration.json) 하나만** 쓴다. 정책이 그 좌표계로 학습됐기 때문이다.

그래서 arm_driver 는 기동할 때 서보의 Homing_Offset 이 이 파일과 **정확히 같은지**
확인하고, 다르면 움직이지 않는다(tools/write_calibration.py 로 맞춘다). 같다는 것이
확인되면 서보가 보고하는 raw 가 곧 LeRobot 의 raw 이므로 보정값(delta)이 필요 없다.

## 공식 (lerobot motors_bus._normalize / _unnormalize 와 동일)

- servo 1..5 DEGREES:   deg = (raw - mid) * 360 / 4095,  mid = (range_min + range_max) / 2
                        raw = int(deg * 4095 / 360 + mid)          ← 자르지 않는다
- servo 6   RANGE_0_100: pct = (clip(raw) - min) / (max - min) * 100
                        raw = int(clip(pct, 0, 100) / 100 * (max - min) + min)
  drive_mode 가 1 이면 pct 를 100 - pct 로 뒤집는다(DEGREES 에는 적용되지 않는다).

⚠️ `int()` 절삭까지 lerobot 과 똑같이 둔다. round 로 바꾸면 학습 때와 1틱씩 다른
목표가 나간다.

⚠️ DEGREES 관절에 range 클램프를 걸지 않는다. lerobot 도 안 건다. 특히 wrist_roll 은
기록 범위가 14틱뿐이라 여기서 자르면 관절이 사실상 고정된다. 절대 한계는 서보의
Min/Max_Position_Limit 레지스터가 잡고, 스텝당 이동량은 arm_driver 가 묶는다.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
SERVO_IDS = (1, 2, 3, 4, 5, 6)
NUM_JOINTS = 6
GRIPPER_INDEX = 5
WRIST_ROLL_INDEX = 4
SHOULDER_LIFT_INDEX = 1

#: STS3215 한 바퀴 4096 카운트. lerobot 정규화 분모는 해상도 - 1 = 4095 다.
STS3215_MAX_RES = 4095
RAW_MIN, RAW_MAX = 0, 4095


class CalibrationError(ValueError):
    pass


@dataclass(frozen=True)
class JointCalibration:
    name: str
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    @property
    def is_gripper(self) -> bool:
        return self.name == "gripper"

    @property
    def mid(self) -> float:
        return (self.range_min + self.range_max) / 2.0


def degrees_to_raw_delta(deg: float) -> float:
    return deg * STS3215_MAX_RES / 360.0


def raw_delta_to_degrees(raw: float) -> float:
    return raw * 360.0 / STS3215_MAX_RES


class ArmCalibration:
    """관절 6개의 캘리브레이션. 순서는 항상 servo 1..6 이다."""

    def __init__(self, joints: Sequence[JointCalibration]) -> None:
        joints = tuple(joints)
        if tuple(j.name for j in joints) != JOINT_NAMES:
            raise CalibrationError(f"관절 이름/순서가 다르다: {[j.name for j in joints]}")
        if tuple(j.id for j in joints) != SERVO_IDS:
            raise CalibrationError(f"servo id 가 1..6 이 아니다: {[j.id for j in joints]}")
        for j in joints:
            if j.range_max <= j.range_min:
                raise CalibrationError(f"{j.name}: range_min({j.range_min}) >= range_max({j.range_max})")
            if abs(j.homing_offset) > 2047:
                raise CalibrationError(f"{j.name}: homing_offset {j.homing_offset} 가 ±2047 밖이다")
            if j.drive_mode not in (0, 1):
                raise CalibrationError(f"{j.name}: drive_mode 는 0/1: {j.drive_mode}")
        self.joints = joints

    # -- 적재 ----------------------------------------------------------------
    @classmethod
    def from_dict(cls, data: dict) -> "ArmCalibration":
        try:
            joints = [
                JointCalibration(
                    name=name,
                    id=int(data[name]["id"]),
                    drive_mode=int(data[name].get("drive_mode", 0)),
                    homing_offset=int(data[name]["homing_offset"]),
                    range_min=int(data[name]["range_min"]),
                    range_max=int(data[name]["range_max"]),
                )
                for name in JOINT_NAMES
            ]
        except (KeyError, TypeError, ValueError) as exc:
            raise CalibrationError(f"캘리브레이션 형식 오류: {exc!r}") from exc
        extra = set(data) - set(JOINT_NAMES)
        if extra:
            raise CalibrationError(f"모르는 관절 이름: {sorted(extra)}")
        return cls(joints)

    @classmethod
    def load(cls, path: str | Path) -> "ArmCalibration":
        path = Path(path)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise CalibrationError(f"캘리브레이션을 읽지 못했다({path}): {exc}") from exc
        return cls.from_dict(data)

    @property
    def homing_offsets(self) -> dict[int, int]:
        return {j.id: j.homing_offset for j in self.joints}

    # -- 변환 ----------------------------------------------------------------
    def raw_to_policy(self, raw6: Sequence[int]) -> list[float]:
        if len(raw6) != NUM_JOINTS:
            raise ValueError(f"raw 는 6개여야 한다: {len(raw6)}")
        out = []
        for j, raw in zip(self.joints, raw6):
            raw = float(raw)
            if j.is_gripper:
                bounded = min(j.range_max, max(j.range_min, raw))
                pct = (bounded - j.range_min) / (j.range_max - j.range_min) * 100.0
                out.append(100.0 - pct if j.drive_mode else pct)
            else:
                out.append((raw - j.mid) * 360.0 / STS3215_MAX_RES)
        return out

    def policy_to_raw(self, values6: Sequence[float]) -> list[int]:
        if len(values6) != NUM_JOINTS:
            raise ValueError(f"값은 6개여야 한다: {len(values6)}")
        out = []
        for j, value in zip(self.joints, values6):
            value = float(value)
            if j.is_gripper:
                if j.drive_mode:
                    value = 100.0 - value
                bounded = min(100.0, max(0.0, value))
                raw = int((bounded / 100.0) * (j.range_max - j.range_min) + j.range_min)
            else:
                raw = int(value * STS3215_MAX_RES / 360.0 + j.mid)
            out.append(raw)
        return out

    def gripper_percent_to_raw(self, percent: float) -> int:
        j = self.joints[GRIPPER_INDEX]
        value = 100.0 - percent if j.drive_mode else percent
        bounded = min(100.0, max(0.0, float(value)))
        return int((bounded / 100.0) * (j.range_max - j.range_min) + j.range_min)
