import numpy as np
import pytest

from vla_common import policy_wire as pw


def test_request_roundtrip():
    img = (np.arange(180 * 320 * 3) % 251).astype(np.uint8).reshape(180, 320, 3)
    body = pw.encode_request(img, [1, 2, 3, 4, 5, 6], "pick up the queen")
    got, state, task = pw.decode_request(body)
    assert np.array_equal(got, img)
    assert state == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert task == "pick up the queen"


def test_response_roundtrip():
    chunk = np.random.default_rng(0).normal(size=(100, 6)).astype(np.float32)
    got, ms = pw.decode_response(pw.encode_response(chunk, 12.34))
    assert np.array_equal(got, chunk)
    assert ms == pytest.approx(12.3)


def test_error_is_raised():
    with pytest.raises(pw.WireError, match="boom"):
        pw.decode_response(pw.encode_error("boom"))


def test_truncated_pixels_rejected():
    img = np.zeros((4, 4, 3), np.uint8)
    body = pw.encode_request(img, [0] * 6, "")
    with pytest.raises(pw.WireError):
        pw.decode_request(body[:-1])
