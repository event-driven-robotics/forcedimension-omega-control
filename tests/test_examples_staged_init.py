"""Hardware-free checks for example code and the staged initialization contract.

These tests parse source only; they never import the hardware SDK, open a
device, or run an example. Physical behavior still requires supervision.
"""
import ast
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "omega_control" / "scripts"
INITIALIZING_EXAMPLES = (
    "data_recording_example.py",
    "motion_example.py",
    "motion_example_periodic.py",
    "motion_parameters_example.py",
    "preload_example.py",
    "read_state_example.py",
    "trajectory_8_example.py",
    "trajectory_circle_example.py",
    "trajectory_example.py",
)


def _call_name(call):
    if isinstance(call.func, ast.Name):
        return call.func.id
    if isinstance(call.func, ast.Attribute):
        return call.func.attr
    return None


def _robot_method_call(node, name):
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "robot"
        and node.func.attr == name
    )


def _keywords(call):
    return {keyword.arg: keyword.value for keyword in call.keywords}


@pytest.mark.parametrize("filename", INITIALIZING_EXAMPLES)
def test_initializing_examples_use_staged_drd_index_and_serial(filename):
    tree = ast.parse((SCRIPT_DIR / filename).read_text(encoding="utf-8"))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    constructors = [
        call for call in calls if _call_name(call) == "OmegaRobot"
    ]
    assert len(constructors) == 1
    constructor_kwargs = _keywords(constructors[0])
    assert isinstance(constructor_kwargs.get("auto_open"), ast.Constant)
    assert constructor_kwargs["auto_open"].value is False
    assert isinstance(constructor_kwargs.get("auto_initialize"), ast.Constant)
    assert constructor_kwargs["auto_initialize"].value is False

    opens = [call for call in calls if _robot_method_call(call, "open")]
    inits = [call for call in calls if _robot_method_call(call, "initialize")]
    assert len(opens) == 1
    assert len(inits) == 1
    assert opens[0].lineno < inits[0].lineno

    open_kwargs = _keywords(opens[0])
    assert {"device_index", "expected_serial"}.issubset(open_kwargs)
    assert isinstance(open_kwargs["device_index"], ast.Attribute)
    assert open_kwargs["device_index"].attr == "device_index"
    assert isinstance(open_kwargs["expected_serial"], ast.Attribute)
    assert open_kwargs["expected_serial"].attr == "serial"

    parsers = [call for call in calls if _call_name(call) == "add_argument"]
    required_flags = {
        args[0].value
        for call in parsers
        if (args := call.args)
        and isinstance(args[0], ast.Constant)
        and isinstance(args[0].value, str)
        and isinstance(_keywords(call).get("required"), ast.Constant)
        and _keywords(call)["required"].value is True
    }
    assert {"--device-index", "--serial"}.issubset(required_flags)


def test_device_info_never_initializes_or_commands_movement():
    tree = ast.parse(
        (SCRIPT_DIR / "device_info_example.py").read_text(encoding="utf-8")
    )
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    constructors = [call for call in calls if _call_name(call) == "OmegaRobot"]
    assert len(constructors) == 1
    kwargs = _keywords(constructors[0])
    assert kwargs["auto_open"].value is False
    assert kwargs["auto_initialize"].value is False
    assert any(_robot_method_call(call, "open") for call in calls)
    forbidden = ("initialize", "force_init", "check_init", "set_pos", "to_origin")
    assert not any(
        _robot_method_call(call, name) for call in calls for name in forbidden
    )
