"""Launch files reuse bringup controller yaml (no sysid-only hardware config)."""

from pathlib import Path


def test_hardware_launch_uses_bringup_controller_yaml() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'excitation_hardware.launch.py'
    )
    text = launch_path.read_text(encoding='utf-8')
    assert "FindPackageShare('open_manipulator_bringup')" in text
    assert 'hardware_controller_manager.yaml' in text
    assert 'controller_manager_hardware.yaml' not in text
    assert 'hardware_torque_enable' not in text
    assert 'measured_torque_publisher' not in text
    assert 'robot_description.py' not in text


def test_gazebo_launch_uses_bringup_xacro_without_sysid_yaml_swap() -> None:
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'excitation_gazebo.launch.py'
    )
    text = launch_path.read_text(encoding='utf-8')
    assert "xacro.process_file(xacro_file, mappings={'use_sim': 'true'})" in text
    assert 'open_manipulator_sysid.robot_description' not in text
    assert 'controller_manager.yaml' not in text
    assert (Path(__file__).resolve().parents[1] / 'config' / 'controller_manager.yaml').exists() is False
