import threading

from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_perception import FakePerception
from domain.task.mission_task import MissionTask, Ports


def _ports(estop=None):
    return Ports(
        base=FakeBase(),
        arm=FakeArm(),
        perception=FakePerception(),
        estop=estop or threading.Event(),
    )


def test_full_mission_completes():
    states = [s.name for s in MissionTask(_ports()).run()]
    assert states[0] == "IDLE"
    assert states[-1] == "RELEASE"
    assert "ESTOP" not in states


def test_estop_interrupts_immediately():
    estop = threading.Event()
    estop.set()
    states = [s.name for s in MissionTask(_ports(estop)).run()]
    assert states[-1] == "ESTOP"


def test_identify_exhausts_retries_and_fails():
    ports = _ports()
    ports.perception = FakePerception(found=False)
    states = [s.name for s in MissionTask(ports).run()]
    assert states.count("IDENTIFY") == 4  # retries=0,1,2,3
    assert states[-1] == "IDENTIFY_FAILED"


def test_grasp_failure_stops_mission():
    ports = _ports()
    ports.arm = FakeArm(move_ok=False)
    states = [s.name for s in MissionTask(ports).run()]
    assert "GRASP" in states
    assert states[-1] == "GRASP_FAILED"
    assert "POSE_PLAN" not in states  # GRASP 실패 이후로는 못 넘어감


def test_narrow_exit_contact_risk_stops_and_fails():
    ports = _ports()
    ports.perception = FakePerception(contact_risk=True)
    states = [s.name for s in MissionTask(ports).run()]
    assert states[-1] == "NARROW_EXIT_FAILED"
    assert "RETURN" not in states  # 접촉 위험 감지 시 RETURN으로 못 넘어감


def test_estop_mid_mission_interrupts_immediately():
    estop = threading.Event()
    ports = _ports(estop)
    gen = MissionTask(ports).run()

    first = next(gen)
    assert first.name == "IDLE"

    estop.set()  # 미션 중간에 E-STOP 발생
    remaining = [s.name for s in gen]
    assert remaining[0] == "ESTOP"
    assert remaining[-1] == "ESTOP"  # ESTOP 이후로는 더 진행 안 됨


def test_base_never_arrives_exhausts_retries_and_fails():
    ports = _ports()
    ports.base = FakeBase(arrive=False)
    states = [s.name for s in MissionTask(ports).run()]
    assert states.count("TRANSIT_OUT") == 6  # retries=0..5
    assert states[-1] == "TRANSIT_OUT_FAILED"
