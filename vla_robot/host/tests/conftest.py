import sys
from pathlib import Path

import pytest

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

import host_config  # noqa: E402  (vla_common 경로 보정)


@pytest.fixture(scope="session")
def cfg():
    return host_config.load_host_config()
