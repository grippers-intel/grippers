import threading
from domain.task.mission_task import MissionTask, Ports
from domain.adapters.fake.fake_base import FakeBase
from domain.adapters.fake.fake_arm import FakeArm
from domain.adapters.fake.fake_perception import FakePerception


def _ports(estop=None):
    return Ports(
        base=FakeBase(), arm=FakeArm(), perception=FakePerception(),
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
