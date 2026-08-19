"""ONNX → HEF 컴파일 호스트가 Hailo Dataflow Compiler 요구사항을 만족하는지 본다.

hld.md 미결 #11 을 판정하는 도구다. DFC 는 **x86_64 Ubuntu 전용**이라
Pi 에서 돌릴 수 없고, 어느 PC 에서 컴파일할지가 M2-H1(#104) 의 선행 조건이다.

    python tools/hailo/check_dfc_env.py

DFC 자체는 PyPI 에 없다 — Hailo Developer Zone 에서 받은 휠을 가상환경에
설치해야 한다(계정 필요). 이 스크립트는 **휠이 없어도** 호스트 적격 여부를
먼저 판정하고, 설치돼 있으면 버전 짝까지 본다.
"""

from __future__ import annotations

import os
import platform
import shutil
import sys

# Hailo 공식 요구사항 (DFC 5.x 기준)
REQUIRED_ARCH = "x86_64"
SUPPORTED_UBUNTU = ("22.04", "24.04")
SUPPORTED_PY = ((3, 10), (3, 11), (3, 12))
MIN_RAM_GB = 16
MIN_FREE_GB = 40  # DFC + 의존성 + 중간 산출물

TARGET_HAILORT = "5.1.1"  # 컨테이너·호스트 드라이버 기준 (PR #151)
TARGET_DEVICE = "HAILO10H"

OK, WARN, FAIL = "PASS", "WARN", "FAIL"


def _ubuntu_version() -> str | None:
    try:
        with open("/etc/os-release", encoding="utf-8") as f:
            info = dict(line.rstrip().split("=", 1) for line in f if "=" in line)
    except OSError:
        return None
    if info.get("ID", "").strip('"') != "ubuntu":
        return None
    return info.get("VERSION_ID", "").strip('"')


def _ram_gb() -> float | None:
    try:
        return os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 1024**3
    except (ValueError, OSError):
        return None


def _dfc_version() -> str | None:
    """설치돼 있으면 DFC 버전. 없으면 None."""
    try:
        from hailo_sdk_client import __version__  # type: ignore[import-not-found]

        return str(__version__)
    except Exception:
        return None


def _checks() -> list[tuple[str, str, str]]:
    """(항목, 판정, 설명) 목록."""
    out: list[tuple[str, str, str]] = []

    arch = platform.machine()
    out.append(
        (
            "아키텍처",
            OK if arch == REQUIRED_ARCH else FAIL,
            f"{arch} (요구 {REQUIRED_ARCH} — DFC 는 ARM 미지원이라 Pi 에서 못 돌린다)",
        )
    )

    ver = _ubuntu_version()
    if ver is None:
        out.append(("OS", WARN, "Ubuntu 가 아니다 — WSL2 나 Docker 로 우회해야 한다"))
    else:
        out.append(
            (
                "OS",
                OK if ver in SUPPORTED_UBUNTU else WARN,
                f"Ubuntu {ver} (지원 {' · '.join(SUPPORTED_UBUNTU)})",
            )
        )

    py = sys.version_info[:2]
    out.append(
        (
            "Python",
            OK if py in SUPPORTED_PY else FAIL,
            f"{py[0]}.{py[1]} (지원 " + " · ".join(f"{a}.{b}" for a, b in SUPPORTED_PY) + ")",
        )
    )

    ram = _ram_gb()
    if ram is None:
        out.append(("RAM", WARN, "측정 실패"))
    else:
        out.append(
            ("RAM", OK if ram >= MIN_RAM_GB else FAIL, f"{ram:.0f} GB (요구 {MIN_RAM_GB} GB↑)")
        )

    free = shutil.disk_usage(os.path.expanduser("~")).free / 1024**3
    out.append(
        (
            "디스크 여유",
            OK if free >= MIN_FREE_GB else WARN,
            f"{free:.0f} GB (권장 {MIN_FREE_GB} GB↑)",
        )
    )

    # 이 저장소 환경 특유의 함정: ROS 를 source 하면 PYTHONPATH 가 venv 로 샌다.
    leak = os.environ.get("PYTHONPATH", "")
    out.append(
        (
            "PYTHONPATH",
            OK if not leak else FAIL,
            (
                "비어 있음"
                if not leak
                else f"{leak} — venv 안으로 새어 들어온다. DFC 는 numpy 등을 고정 버전으로 "
                "요구하므로 ROS 패키지와 충돌한다. **DFC 셸에서는 `unset PYTHONPATH`**"
            ),
        )
    )

    dfc = _dfc_version()
    if dfc is None:
        out.append(("DFC", WARN, "미설치 — PyPI 에 없다. Developer Zone 휠을 이 venv 에 설치할 것"))
    else:
        out.append(("DFC", OK, f"{dfc} 설치됨"))
        out.append(
            (
                "버전 짝",
                WARN,
                f"DFC {dfc} 가 HailoRT {TARGET_HAILORT} / {TARGET_DEVICE} 와 짝인지 "
                "릴리스 노트로 확인할 것 — 어긋나면 HEF 가 런타임에서 로드되지 않는다",
            )
        )

    return out


def main() -> int:
    print(f"Hailo DFC 호스트 적격성 — 대상 {TARGET_DEVICE} · HailoRT {TARGET_HAILORT}\n")
    rows = _checks()
    width = max(len(name) for name, _, _ in rows)
    for name, verdict, note in rows:
        print(f"  [{verdict:4}] {name:<{width}}  {note}")

    failed = [n for n, v, _ in rows if v == FAIL]
    print()
    if failed:
        print(f"부적격 — {', '.join(failed)} 을(를) 먼저 해결해야 한다.")
        return 1
    if any(v == WARN for _, v, _ in rows):
        print("호스트 요구사항은 만족한다. WARN 항목은 컴파일 전에 확인할 것.")
        return 0
    print("모두 통과 — 컴파일 가능.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
