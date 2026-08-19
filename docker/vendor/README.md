# docker/vendor — HailoRT 벤더 바이너리

이 디렉터리의 `.deb` · `.whl` 파일은 **Git에 포함하지 않습니다**
(`.gitignore`).

HailoRT가 필요한 Raspberry Pi에서는 아래 두 파일을 직접 배치한 뒤
Docker image를 빌드합니다.

두 파일이 없더라도 Docker build 자체는 정상적으로 완료되며,
그 경우 HailoRT 지원만 생략됩니다.

## 필요한 파일

| 파일명 | 출처 |
|---|---|
| `h10-hailort_5.1.1_arm64.deb` | Raspberry Pi Debian 저장소 (`archive.raspberrypi.com/debian trixie`) |
| `hailort-5.1.1-cp310-cp310-linux_aarch64.whl` | Hailo Developer Zone (계정 필요) |

### Native runtime

Raspberry Pi 호스트에서:

```bash
cd docker/vendor
apt download h10-hailort
```

현재 요구 파일명:

`h10-hailort_5.1.1_arm64.deb`

`.deb` 전체를 컨테이너 패키지로 설치하지 않고 다음 userspace 파일만
Docker image에 추출합니다.

- `libhailort.so.5.1.1`
- `hailortcli`

커널 드라이버는 호스트의 `h10-hailort-pcie-driver`를 사용하고,
컨테이너 실행 시 `/dev` bind mount를 통해 Hailo 장치에 접근합니다.

### Python binding

Hailo Developer Zone에서 다음 조건과 정확히 일치하는 wheel을 받습니다.

- HailoRT: `5.1.1`
- Python: `3.10` / `cp310`
- Architecture: `aarch64`

파일명:

`hailort-5.1.1-cp310-cp310-linux_aarch64.whl`

## ⚠️ 버전을 임의로 올리지 말 것

호스트 PCIe driver, 컨테이너 native runtime, Python binding의 HailoRT
버전은 서로 일치해야 합니다.

현재 Pi에서 검증한 조합은 다음과 같습니다.

- Host PCIe driver: `h10-hailort-pcie-driver 5.1.1`
- Native runtime: `h10-hailort 5.1.1`
- Python binding: `hailort 5.1.1 / cp310 / aarch64`
- Container Python: `Python 3.10.12`

## Docker build 동작

vendor 파일 두 개가 모두 존재하면 HailoRT를 설치합니다.

`[hailo] HailoRT 5.1.1 installed`

파일이 하나라도 없으면 HailoRT 설치만 건너뛰고 image build는 계속합니다.

`[hailo] vendor .deb/.whl not found; building without HailoRT`

Dockerfile은 BuildKit `RUN --mount=type=bind`를 사용하므로 `.deb`와 `.whl`
원본은 Docker image layer에 복사되지 않습니다.

실제 HailoRT 연동 검증 결과는 `docs/design/hld.md` §9 #14를 참고하세요.
