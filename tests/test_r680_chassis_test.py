import sys
from pathlib import Path

import pytest


DEPLOYMENT_DIR = Path(__file__).resolve().parents[1] / "deployment"
sys.path.insert(0, str(DEPLOYMENT_DIR))

from r680_chassis_test import (  # noqa: E402
    MAX_DURATION_S,
    MAX_TEST_VX_MPS,
    MAX_TEST_WZ_RADPS,
    validate_test_request,
)


def test_accepts_practical_raised_chassis_motion_request():
    request = validate_test_request(0.08, 0.08, 3.0)

    assert request.vx_m_s == 0.08
    assert request.wz_rad_s == 0.08
    assert request.duration_s == 3.0


def test_rejects_request_outside_raised_chassis_safety_envelope():
    with pytest.raises(ValueError, match="vx"):
        validate_test_request(MAX_TEST_VX_MPS + 0.001, 0.0, 1.0)
    with pytest.raises(ValueError, match="wz"):
        validate_test_request(0.02, MAX_TEST_WZ_RADPS + 0.001, 1.0)
    with pytest.raises(ValueError):
        validate_test_request(0.02, 0.0, MAX_DURATION_S + 0.1)

