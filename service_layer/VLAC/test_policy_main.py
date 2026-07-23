import base64
import inspect
import math

import pytest

import policy_main


PNG_8PX = base64.b64encode(
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x08\x00\x00\x00\x08"
    b"\x08\x02\x00\x00\x00Km)\xdc\x00\x00\x00\x13IDATx\x9cc`\xa0\x1c`\x0c\xa4"
    b"\x86Q\rCH\x03\x00@\xd8\x00\x09\xd0\xd3r\x89\x00\x00\x00\x00IEND\xaeB`\x82"
).decode("ascii")


def test_parse_reliability_is_not_model_confidence():
    parsed, parse_reliability, warnings = policy_main._parse_raw_actions(
        "{x: 1.0mm, y: 2.0mm, z: 3.0mm, roll: 4.0 degrees, pitch: 5.0 degrees, yaw: 6.0 degrees, open: 0.7}"
    )
    assert warnings == []
    assert parse_reliability == "exact_grammar_match"
    assert parsed[0].parse_reliability == "exact_grammar_match"
    assert parsed[0].model_confidence is None
    assert parsed[0].execution_allowed is False


def test_action_semantics_remain_unknown():
    parsed, _, _ = policy_main._parse_raw_actions(
        "{x: 1.0mm, y: 2.0mm, z: 3.0mm, roll: 4.0 degrees, pitch: 5.0 degrees, yaw: 6.0 degrees, open: 0.7}"
    )
    assert parsed[0].action_semantics == "unknown_songling_convention"


def test_gripper_placeholder_is_used_for_model_input():
    state = policy_main.EndEffectorState(
        x_m=0.1,
        y_m=0.2,
        z_m=0.3,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        gripper_m=0.123,
    )
    legacy = policy_main._si_to_legacy_model_units(state)
    assert legacy[-1] == pytest.approx(0.0)


def test_remote_image_urls_rejected_by_default():
    with pytest.raises(Exception) as exc:
        policy_main._normalize_image_input("https://example.com/test.jpg")
    assert "disabled by default" in str(exc.value)


def test_base64_images_still_work():
    image = policy_main._normalize_image_input("data:image/png;base64," + PNG_8PX)
    assert image.width == 8
    assert image.height == 8


def test_numeric_fallback_is_parser_only():
    parsed, parse_reliability, warnings = policy_main._parse_raw_actions("action [1 2 3 4 5 6 7]")
    assert parse_reliability == "assumed_numeric_layout"
    assert parsed[0].raw_translation_m == pytest.approx([0.001, 0.002, 0.003])
    assert parsed[0].raw_rotation_rad == pytest.approx([math.radians(4), math.radians(5), math.radians(6)])
    assert parsed[0].model_confidence is None
    assert warnings


def test_no_robot_moveit_or_can_executor_imports():
    source = inspect.getsource(policy_main)
    lowered = source.lower()
    assert "moveit" not in lowered
    assert "candump" not in lowered
    assert "socketcan" not in lowered
    assert "rospy" not in lowered
