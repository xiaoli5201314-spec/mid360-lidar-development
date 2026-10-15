from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
DEPLOY = PROJECT / "deployment"


def test_web_control_files_are_in_release_manifest():
    manifest = (DEPLOY / "MANIFEST.md").read_text(encoding="utf-8")
    assert "r680_web_chassis_control.py" in manifest
    assert "r680_web_chassis_control.sh" in manifest


def test_web_wrapper_is_explicitly_armed_and_execs_python_entrypoint():
    wrapper = (DEPLOY / "r680_web_chassis_control.sh").read_text(encoding="utf-8")
    assert "set -euo pipefail" in wrapper
    assert "--arm" in wrapper
    assert "exec python3 -u /root/r680_nav/scripts/r680_web_chassis_control.py" in wrapper
    assert "systemctl" not in wrapper
    assert "ros2 topic pub" not in wrapper


def test_nav2_bridge_holds_the_same_global_control_lock():
    start = (DEPLOY / "r680_nav2_chassis_bridge_start.sh").read_text(encoding="utf-8")
    assert "--control-lock /root/r680_nav/run/r680_chassis_control.lock" in start


def test_nav2_real_start_rejects_web_control_lock_before_hardware_startup():
    start = (DEPLOY / "r680_nav2_chassis_start.sh").read_text(encoding="utf-8")
    assert "reject_manual_controller" in start
    assert "/root/r680_nav/run/r680_chassis_control.lock" in start
    main = start.split("# Main workflow", 1)[1]
    assert main.index("reject_manual_controller") < main.index("systemctl start")


def test_web_backend_uses_stable_serial_path_and_current_v4_limits():
    backend = (DEPLOY / "r680_web_chassis_control.py").read_text(encoding="utf-8")
    assert "/dev/serial/by-id/YOUR_CHASSIS_SERIAL" in backend
    assert "MAX_SPEED = 0.26" in backend
    assert "DEFAULT_SPEED = 0.09" in backend
    assert "MINIMUM_TURNING_RADIUS = 1.40" in backend
