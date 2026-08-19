# ONNX → HEF 컴파일 (DFC) 런북

`#104` (M2-H1) 의 선행 조건인 **컴파일 호스트**를 정리한 문서다.
`hld.md` 미결 #11 의 "컴파일 호스트 확보" 조건을 만족시키기 위한 런북이며,
**hld 본문 갱신은 #151 머지 후 별도로 반영한다.**

```bash
python tools/hailo/check_dfc_env.py
```

---

## 0. 왜 별도 호스트가 필요한가

**DFC 는 x86_64 Ubuntu 전용이라 Pi 5 에서 돌릴 수 없다.** ARM 빌드가 없다.
그래서 학습·컴파일은 x86 에서 하고, 산출물 `.hef` 만 Pi 로 옮긴다.

```
x86 호스트                          Pi 5
ONNX ──DFC──► HEF   ──── scp ────►  HailoRT 5.1.1 런타임 (PR #151)
```

## 1. 호스트 요구사항

| 항목 | 요구 | 근거 |
|---|---|---|
| 아키텍처 | **x86_64** | ARM 빌드 없음 |
| OS | Ubuntu **22.04 / 24.04** | Hailo 공식 지원 목록 |
| Python | **3.10 / 3.11 / 3.12** | 〃 |
| RAM | 16 GB↑ | 컴파일 중 그래프 최적화 |
| 디스크 | 40 GB↑ | DFC + 의존성 + 중간 산출물 |
| GPU | 선택 | 일부 최적화에만 쓰인다. 없어도 컴파일된다 |

> [!IMPORTANT]
> **호스트는 아직 정해지지 않았다** (hld 미결 #11).
> 후보가 될 PC 에서 위 스크립트를 돌려 적격 여부를 보고하는 것이 이 문서의 용도다.
> 결정은 별도 이슈에서 한다.

## 2. ⚠️ PYTHONPATH — DFC 셸에서는 비운다

**정책: DFC 작업 셸에서는 외부 `PYTHONPATH` 를 일절 허용하지 않는다.**
무엇이 들어 있든 venv 격리를 깨기 때문이고, `check_dfc_env.py` 도 값이 있으면 FAIL 로 잡는다.

가장 흔한 원인은 ROS 다. 셸이 `source /opt/ros/<distro>/setup.bash` 를 실행하면
**`PYTHONPATH` 가 새로 만든 venv 안까지 샌다.** ROS 를 쓰는 개발 PC 라면 대개 그렇다.

venv 를 새로 만들어도 ROS 패키지가 보인다. DFC 는 numpy 등을 고정 버전으로 요구해서
ROS Jazzy 가 깔아둔 것과 충돌한다. **DFC 를 쓰는 셸에서는 반드시 끊는다.**

```bash
unset PYTHONPATH        # 또는 env -u PYTHONPATH <명령>
```

`check_dfc_env.py` 가 이걸 FAIL 로 잡는다. 오염된 셸에서 설치하면
증상이 설치 시점이 아니라 **컴파일 도중**에 나와 원인 찾기가 어렵다.

## 3. 설치

DFC 는 **PyPI 에 없다** — `pip install hailo-dataflow-compiler` 는 404 다.
Hailo Developer Zone(계정 필요)에서 휠을 받아야 한다.

```bash
unset PYTHONPATH
cd ~/hailo-dfc
python3.12 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip wheel setuptools
pip install vendor/hailo_dataflow_compiler-<버전>-py3-none-linux_x86_64.whl
python tools/hailo/check_dfc_env.py     # DFC 줄이 PASS 로 바뀌는지 확인
```

작업 트리는 저장소 밖에 둔다 — 휠과 중간 산출물이 크고 재배포 불가다. 예:

```
~/hailo-dfc/
├── .venv/     DFC 전용 (PYTHONPATH 오염 없이 생성)
├── vendor/    DFC 휠 (Git 밖)
├── onnx/      입력
├── hef/       산출물 → Pi 로 scp
└── logs/      컴파일 로그
```

## 4. ⚠️ 호환 조합을 먼저 확인한다

**DFC 와 런타임 HailoRT 버전이 어긋나면 HEF 가 Pi 에서 로드되지 않는다.**
컴파일은 성공하고 런타임에서만 실패해서, 모르면 엉뚱한 곳을 판다.

현재 런타임은 **HailoRT 5.1.1 · HAILO10H** 로 고정돼 있다(PR #151, `docker/vendor/README.md`).
Developer Zone 릴리스 노트에서 **HailoRT 5.1.1 / HAILO10H 와 호환되는 DFC 조합**을
확인하고 그것을 받는다. 컴파일 타깃도 `HAILO10H` 로 지정해야 한다.

> [!NOTE]
> `check_dfc_env.py` 는 **호환 여부를 판정하지 않는다.** 설치된 DFC 버전을 표시하고
> 확인하라고 요구할 뿐이다 — 공식 조합표를 아직 확보하지 못했다. 확보하면 판정으로 올린다.

## 5. 남은 선행 조건

| | 상태 |
|---|---|
| 컴파일 호스트 | ⛔ **미정** — 후보 PC 에서 위 스크립트로 자가진단 후 결정 (미결 #11) |
| DFC 휠 | ⛔ Developer Zone 계정 필요 |
| 컴파일할 ONNX | ⛔ #102 (M2-D2) YOLO 학습 결과 대기 |

**세 번째가 진짜 임계 경로다.** 호스트와 휠이 준비돼도 컴파일할 모델이 없으면 못 돈다.
`.hef` 는 최종 후보 1~2 개에만 만든다 — 탐색은 ONNX 로 반복한다.
