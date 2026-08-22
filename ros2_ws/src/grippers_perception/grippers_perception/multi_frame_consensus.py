"""바닥 스캔 다중 프레임 합의 필터 (순수 함수).

미션 명세서(2026-08-23) 실행 파이프라인 04번 — "베이스를 세운 뒤 여러 프레임을
모아, 반복해서 보인 것만 인정하고 위치는 중앙값을 씁니다." 파지 성공률을
좌우하는 단계로 지목됐고, 명세서의 "작업 순서"에서도 1번(최우선)이다 —
"bag만 있으면 개발용 Pi·노트북에서 병렬 개발 가능"이라고 명시돼 있어서,
`hailo_scan_mapping.py`/`cpu_yolo_scan_mapping.py`와 같은 이유로 이 로직만
따로 뽑는다: perception_node.py는 rclpy를 무조건 import해서 ROS2 없이는
임포트가 안 되지만, 이 파일은 그 무엇도 import하지 않으므로 순수 pytest로
검증할 수 있고 하드웨어/ROS 환경 없이 그대로 개발·테스트할 수 있다.

입력·출력 모두 ROS 메시지 타입에 의존하지 않는다. 정지 상태에서 여러
프레임을 얻었다는 가정 하에, 각 프레임의 원시 검출(`RawDetection`) 리스트를
모아 클래스별로 클러스터링하고, `k_of_n` 프레임 이상에서 반복 관측된
클러스터만 인정한다. 최종 위치는 (미션 명세서 지시대로) 그 클러스터
멤버들의 x·y 각각의 **중앙값**이다.

⚠️ 아직 실기로 검증 안 됨: `DEFAULT_CLUSTER_RADIUS_M`·`DEFAULT_K_OF_N`과
프레임 수 N은 전부 자리표시자다 — 실측 노이즈 폭(같은 물체를 프레임마다
얼마나 다른 좌표로 보는지)을 재서 다시 잡을 것. 프레임 사이 로봇·물체가
완전히 정지해 있다는 가정도 실기로 아직 확인 안 됐다 — 베이스가 완전히
멈춘 뒤에만 이 필터를 호출해야 한다(호출자 책임)."""

from dataclasses import dataclass, field
from statistics import median

# ── 자리표시자 — 실측 전 ────────────────────────────────────────────────
# 두 검출을 "같은 물체"로 볼 최대 거리(m). 실측 프레임 간 노이즈 폭보다는
# 커야 하고, 미션 명세서가 요구하는 "물체 간격을 넓게 배치"(파이프라인 06
# 축소안)로 보장될 물체 간 최소 간격보다는 작아야 한다 — 둘 다 실측 전 근사.
DEFAULT_CLUSTER_RADIUS_M = 0.05
# N프레임 중 최소 몇 프레임에서 봐야 인정할지. 미션 명세서는 구체적 숫자를
# 정하지 않았다 — "반복해서 보인 것만 인정한다"는 원칙만 확정돼 있다.
DEFAULT_K_OF_N = 3


@dataclass(frozen=True)
class RawDetection:
    """한 프레임에서 나온 원시 검출 하나.

    ROS `Detection` 메시지가 아니라 perception_node.py가 스캔 프레임마다
    만드는 중간 표현이다 — 클러스터링이 끝난 뒤에야(`ConsensusDetection`)
    최종 `Detection` 메시지(track_id 포함)로 바뀐다."""

    class_key: str  # 도메인 ObjectClass 문자열("GABE"/"CHESS_PIECE") 등 — 클러스터링 키
    x: float
    y: float
    yaw_rad: float
    confidence: float


@dataclass(frozen=True)
class ConsensusDetection:
    """`k_of_n` 이상 프레임에서 반복 관측돼 인정된 클러스터의 합의 결과."""

    class_key: str
    x: float
    y: float
    yaw_rad: float
    confidence: float
    frames_seen: int


@dataclass
class _Cluster:
    class_key: str
    xs: list = field(default_factory=list)
    ys: list = field(default_factory=list)
    yaws: list = field(default_factory=list)
    confidences: list = field(default_factory=list)

    @property
    def frames_seen(self):
        return len(self.xs)

    def centroid(self):
        return sum(self.xs) / len(self.xs), sum(self.ys) / len(self.ys)

    def add(self, det):
        self.xs.append(det.x)
        self.ys.append(det.y)
        self.yaws.append(det.yaw_rad)
        self.confidences.append(det.confidence)

    def to_consensus(self):
        return ConsensusDetection(
            class_key=self.class_key,
            x=median(self.xs),
            y=median(self.ys),
            yaw_rad=median(self.yaws),
            confidence=max(self.confidences),
            frames_seen=self.frames_seen,
        )


def _distance(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def consensus_detections(frames, k_of_n=DEFAULT_K_OF_N, cluster_radius_m=DEFAULT_CLUSTER_RADIUS_M):
    """여러 프레임의 원시 검출을 합의해 최종 검출 목록으로 줄인다.

    `frames`는 프레임별 `RawDetection` 리스트의 리스트(관측 순서대로 — 이
    순서를 보장하는 건 호출자 책임이다). 한 프레임 안의 검출끼리는 서로
    다른 물체라고 가정한다 — 같은 프레임의 두 검출을 한 클러스터로 합치지
    않는다(중복 계수 방지). 프레임을 하나씩 처리하면서, 그 프레임에서
    아직 매칭되지 않은 기존 클러스터 중 같은 `class_key` + `cluster_radius_m`
    이내로 가장 가까운 것에 매칭하는 그리디 최근접 방식을 쓴다. 매칭되는
    클러스터가 없으면 새 클러스터를 연다.

    `k_of_n` 미만 프레임에서만 관측된 클러스터는 오검출/일시적 노이즈로
    보고 버린다 — 결과에 남지 않는다(다른 관측 포트와 같은 "모르면 제외"
    관례). 위치(x, y)와 yaw는 멤버들의 **중앙값**(미션 명세서 지시), confidence는
    **최댓값**(그 물체를 가장 확신 있게 봤던 순간을 대표값으로 삼는다)이다."""
    clusters = []

    for frame in frames:
        claimed = set()
        for det in frame:
            best_idx = None
            best_dist = cluster_radius_m
            for idx, cluster in enumerate(clusters):
                if idx in claimed or cluster.class_key != det.class_key:
                    continue
                cx, cy = cluster.centroid()
                dist = _distance(cx, cy, det.x, det.y)
                if dist <= best_dist:
                    best_dist = dist
                    best_idx = idx
            if best_idx is None:
                clusters.append(_Cluster(class_key=det.class_key))
                best_idx = len(clusters) - 1
            clusters[best_idx].add(det)
            claimed.add(best_idx)

    return [c.to_consensus() for c in clusters if c.frames_seen >= k_of_n]
