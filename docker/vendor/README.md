# docker/vendor — HailoRT 벤더 바이너리

이 디렉터리의 `.deb` · `.whl` 은 **Git에서 제외**됩니다 (`.gitignore`).
재배포가 불가능한 벤더 바이너리이고, 컨테이너 배포를 전제하지 않기 때문입니다
(`docs/design/hld.md` §9 #1).

`docker/Dockerfile` 은 아래 두 파일을 **정확한 파일명 그대로** 요구합니다.
없으면 빌드가 다음처럼 죽습니다:

```
failed to compute cache key: "/docker/vendor/h10-hailort_5.1.1_arm64.deb": not found
```

## 필요한 파일

| 파일명 | 출처 |
|---|---|
| `h10-hailort_5.1.1_arm64.deb` | Raspberry Pi Debian 저장소 (`archive.raspberrypi.com/debian trixie`) |
| `hailort-5.1.1-cp310-cp310-linux_aarch64.whl` | Hailo Developer Zone (계정 필요) |

`.deb` 에서는 `libhailort.so.5.1.1` 과 `hailortcli` 만 추출해 씁니다.
커널 드라이버는 **호스트의 `h10-hailort-pcie-driver`** 를 그대로 사용하고,
장치 접근은 컨테이너 실행 시 `/dev` bind mount 로 확보합니다.

## ⚠️ 버전을 임의로 올리지 말 것

호스트 커널 드라이버 · 컨테이너 런타임 · Python 바인딩 **셋의 버전이
정확히 일치해야 합니다.** 현재 기준은 **5.1.1** 입니다.

휠은 `cp310` · `aarch64` 여야 합니다 — 컨테이너(Ubuntu 22.04 jammy)의
Python 이 3.10 이라, Developer Zone 기본 제공본인 `cp313` 은 import 되지 않습니다.
이 불일치가 미결 #14 가 오래 막혀 있던 원인이었습니다.

자세한 경위는 `docs/design/hld.md` §9 미결 #14.
