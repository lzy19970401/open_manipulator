"""Rosbag2 → :class:`IdentificationDataset` (read bags, resample, validate)."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS
from open_manipulator_sysid.excitation_recording.dataset import (
    IdentificationDataset,
    filtered_acceleration,
    resample_uniform,
    validate_identification_dataset,
)
# XM430-W350-R/T — same factor as open_manipulator_x_current.ros2_control.xacro.
XM430_PRESENT_CURRENT_NM_MULTIPLIER = 0.00479627
# Stall torque ~4.1 N·m; magnitudes above this are almost certainly Dynamixel LSB.
MAX_REASONABLE_ARM_TORQUE_NM = 5.0

JOINT_STATES_TOPIC = '/joint_states'
ARM_CONTROLLER_STATE_TOPIC = '/arm_controller/controller_state'
EXCITATION_RECORDING_TOPICS = (JOINT_STATES_TOPIC, ARM_CONTROLLER_STATE_TOPIC)


def arm_effort_array_to_nm(efforts: np.ndarray) -> np.ndarray:
    """Normalize ``/joint_states.effort`` array to N·m (hardware LSB or Gazebo N·m)."""
    array = np.asarray(efforts, dtype=float)
    if array.size == 0 or np.max(np.abs(array)) <= MAX_REASONABLE_ARM_TORQUE_NM:
        return array
    return array * XM430_PRESENT_CURRENT_NM_MULTIPLIER


def detect_bag_storage_id(bag_dir: Path) -> str:
    """Return rosbag2 storage plugin id ('mcap' or 'sqlite3') from metadata.yaml."""
    metadata_path = bag_dir / 'metadata.yaml'
    if metadata_path.is_file():
        with metadata_path.open('r', encoding='utf-8') as handle:
            metadata = yaml.safe_load(handle) or {}
        storage_id = (
            metadata.get('rosbag2_bagfile_information', {}).get('storage_identifier')
        )
        if storage_id in ('mcap', 'sqlite3'):
            return storage_id

    if any(bag_dir.glob('*.mcap')):
        return 'mcap'
    if any(bag_dir.glob('*.db3')):
        return 'sqlite3'

    raise ValueError(
        f'Could not determine rosbag2 storage format for {bag_dir}. '
        'Expected metadata.yaml with storage_identifier or *.mcap / *.db3 files.'
    )


def _open_bag_reader(bag_dir: Path):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    storage_id = detect_bag_storage_id(bag_dir)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    return reader


def bag_topic_names(bag_dir: Path | str) -> set[str]:
    """Return topic names present in a rosbag2 directory."""
    reader = _open_bag_reader(Path(bag_dir))
    return {item.name for item in reader.get_all_topics_and_types()}


def require_excitation_recording_bag(bag_dir: Path | str) -> None:
    """Raise if the bag is missing topics required for tracking comparison."""
    topics = bag_topic_names(bag_dir)
    missing = [topic for topic in EXCITATION_RECORDING_TOPICS if topic not in topics]
    if missing:
        joined = ', '.join(missing)
        raise ValueError(
            f'Bag {bag_dir} is missing required topic(s): {joined}. '
            'Re-record with excitation_* or test_trajectory_* launch (record_bag:=true).'
        )


def interpolate_arrays_at_times(
    query_times_s: np.ndarray,
    source_times_s: np.ndarray,
    source_values: np.ndarray,
) -> np.ndarray:
    """Linearly interpolate ``(N, dof)`` samples onto ``query_times_s``."""
    if source_times_s.size < 2:
        raise ValueError('Need at least two source samples for interpolation.')
    if query_times_s.size == 0:
        return np.empty((0, source_values.shape[1]), dtype=float)

    interpolated = np.empty((query_times_s.size, source_values.shape[1]), dtype=float)
    for joint_index in range(source_values.shape[1]):
        interpolated[:, joint_index] = np.interp(
            query_times_s,
            source_times_s,
            source_values[:, joint_index],
        )
    return interpolated


def interpolate_positions_at_times(
    query_times_s: np.ndarray,
    source_times_s: np.ndarray,
    source_positions: np.ndarray,
) -> np.ndarray:
    """Linearly interpolate ``(N, dof)`` positions onto ``query_times_s``."""
    return interpolate_arrays_at_times(
        query_times_s,
        source_times_s,
        source_positions,
    )


def overlap_time_mask(
    query_times_s: np.ndarray,
    source_times_s: np.ndarray,
) -> np.ndarray:
    """Keep query samples that fall within the source time span (inclusive)."""
    if source_times_s.size == 0:
        return np.zeros(query_times_s.shape, dtype=bool)
    start_s = float(source_times_s[0])
    end_s = float(source_times_s[-1])
    return (query_times_s >= start_s) & (query_times_s <= end_s)


def joint_states_from_bag(
    bag_path: Path | str,
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Read /joint_states from a rosbag2 directory."""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise ImportError(
            'rosbag2_py and rclpy are required to read bags. Run inside the ROS 2 container.'
        ) from exc

    bag_dir = Path(bag_path)
    if not bag_dir.is_dir():
        raise FileNotFoundError(f'Bag directory not found: {bag_dir}')

    reader = _open_bag_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    joint_states_type = get_message(topic_types['/joint_states'])

    timestamps: List[int] = []
    positions: List[List[float]] = []
    velocities: List[List[float]] = []
    efforts: List[List[float]] = []

    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        if topic != '/joint_states':
            continue
        message = deserialize_message(payload, joint_states_type)
        name_to_index = {name: index for index, name in enumerate(message.name)}
        if not set(ARM_JOINTS).issubset(name_to_index):
            continue

        timestamps.append(timestamp)
        positions.append([message.position[name_to_index[joint]] for joint in ARM_JOINTS])
        if message.velocity:
            velocities.append(
                [message.velocity[name_to_index[joint]] for joint in ARM_JOINTS]
            )
        else:
            velocities.append([0.0] * len(ARM_JOINTS))
        if message.effort:
            efforts.append([message.effort[name_to_index[joint]] for joint in ARM_JOINTS])
        else:
            efforts.append([0.0] * len(ARM_JOINTS))

    if not timestamps:
        raise ValueError(f'No /joint_states samples with Arm joints found in {bag_dir}')

    times_s = np.asarray(timestamps, dtype=float) * 1e-9
    arrays = {
        'position': np.asarray(positions, dtype=float),
        'velocity': np.asarray(velocities, dtype=float),
        'torque': np.asarray(efforts, dtype=float),
    }

    arrays['torque'] = arm_effort_array_to_nm(arrays['torque'])

    return times_s, arrays


def _reference_joint_values(
    reference,
    *,
    name_to_index: Dict[str, int],
    component: str,
) -> List[float]:
    values = getattr(reference, component, None)
    if not values:
        return [0.0] * len(ARM_JOINTS)
    return [float(values[name_to_index[joint]]) for joint in ARM_JOINTS]


def commanded_reference_from_bag(
    bag_path: Path | str,
    *,
    topic: str = ARM_CONTROLLER_STATE_TOPIC,
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Read commanded q/q̇/q̈ from ``JointTrajectoryControllerState.reference``."""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise ImportError(
            'rosbag2_py and rclpy are required to read bags. Run inside the ROS 2 container.'
        ) from exc

    bag_dir = Path(bag_path)
    if not bag_dir.is_dir():
        raise FileNotFoundError(f'Bag directory not found: {bag_dir}')

    reader = _open_bag_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(
            f'Bag {bag_dir} does not contain {topic}. '
            'Re-record with excitation_* or test_trajectory_* launch (record_bag:=true).'
        )

    state_type = get_message(topic_types[topic])
    timestamps: List[int] = []
    positions: List[List[float]] = []
    velocities: List[List[float]] = []
    accelerations: List[List[float]] = []

    while reader.has_next():
        bag_topic, payload, timestamp = reader.read_next()
        if bag_topic != topic:
            continue
        message = deserialize_message(payload, state_type)
        name_to_index = {name: index for index, name in enumerate(message.joint_names)}
        if not set(ARM_JOINTS).issubset(name_to_index):
            continue
        if not message.reference.positions:
            continue

        timestamps.append(timestamp)
        positions.append(
            _reference_joint_values(
                message.reference,
                name_to_index=name_to_index,
                component='positions',
            )
        )
        velocities.append(
            _reference_joint_values(
                message.reference,
                name_to_index=name_to_index,
                component='velocities',
            )
        )
        accelerations.append(
            _reference_joint_values(
                message.reference,
                name_to_index=name_to_index,
                component='accelerations',
            )
        )

    if not timestamps:
        raise ValueError(
            f'No {topic} samples with arm joint reference positions found in {bag_dir}'
        )

    times_s = np.asarray(timestamps, dtype=float) * 1e-9
    arrays = {
        'position': np.asarray(positions, dtype=float),
        'velocity': np.asarray(velocities, dtype=float),
        'acceleration': np.asarray(accelerations, dtype=float),
    }
    return times_s, arrays


def commanded_positions_from_bag(
    bag_path: Path | str,
    *,
    topic: str = ARM_CONTROLLER_STATE_TOPIC,
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """Read commanded arm positions from ``JointTrajectoryControllerState.reference``."""
    times_s, arrays = commanded_reference_from_bag(bag_path, topic=topic)
    return times_s, {'position': arrays['position']}


def bag_to_dataset(
    bag_path: Path | str,
    *,
    sample_dt_s: float = 0.01,
    smooth_window: int = 11,
    trim_start_s: Optional[float] = None,
    trim_end_s: Optional[float] = None,
    trim_end_reference: str = 'bag_end',
) -> IdentificationDataset:
    """Convert rosbag2 joint_states into a uniform identification dataset.

    ``trim_end_reference``:
    - ``bag_end``: ``trim_end_s`` is cut from the bag's last timestamp (CLI default).
    - ``bag_start``: ``trim_end_s`` is an absolute offset from the bag's first timestamp
      (sysid schedule windows from ``identification_trim_times``).
    """
    times_s, raw = joint_states_from_bag(bag_path)

    if trim_start_s is not None or trim_end_s is not None:
        start_time = times_s[0] + (trim_start_s or 0.0)
        if trim_end_reference == 'bag_start':
            end_time = times_s[0] + (trim_end_s or 0.0)
        elif trim_end_reference == 'bag_end':
            end_time = times_s[-1] - (trim_end_s or 0.0)
        else:
            raise ValueError(
                f"trim_end_reference must be 'bag_start' or 'bag_end', got {trim_end_reference!r}"
            )
        mask = (times_s >= start_time) & (times_s <= end_time)
        times_s = times_s[mask]
        raw = {key: value[mask] for key, value in raw.items()}

    grid, position = resample_uniform(times_s, raw['position'], sample_dt_s=sample_dt_s)
    _, velocity = resample_uniform(times_s, raw['velocity'], sample_dt_s=sample_dt_s)
    _, torque = resample_uniform(times_s, raw['torque'], sample_dt_s=sample_dt_s)
    acceleration = filtered_acceleration(
        velocity,
        grid,
        smooth_window=smooth_window,
    )

    dataset = IdentificationDataset(
        times_s=grid,
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        torque=torque,
        sample_dt_s=sample_dt_s,
    )
    validate_identification_dataset(dataset)
    return dataset


def write_dataset_yaml(path: Path | str, dataset: IdentificationDataset) -> None:
    payload = {
        'description': 'Identification dataset metadata (arrays stored in companion .npz)',
        'arm_joints': list(ARM_JOINTS),
        'num_samples': dataset.num_samples,
        'sample_dt_s': dataset.sample_dt_s,
        'duration_s': float(dataset.times_s[-1] - dataset.times_s[0])
        if dataset.num_samples > 1
        else 0.0,
    }
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def main(args: Optional[Sequence[str]] = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description='Convert rosbag2 /joint_states into sysid dataset (.npz).'
    )
    parser.add_argument('--bag', type=Path, required=True, help='Rosbag2 directory path')
    parser.add_argument(
        '--output',
        type=Path,
        default=None,
        help='Output .npz path (default: results/datasets/<bag_name>.npz)',
    )
    parser.add_argument(
        '--dt',
        type=float,
        default=0.01,
        help='Uniform resample period in seconds (default: 0.01)',
    )
    parser.add_argument(
        '--trim-start',
        type=float,
        default=None,
        help='Trim seconds from bag start after loading',
    )
    parser.add_argument(
        '--trim-end',
        type=float,
        default=None,
        help='Trim seconds from bag end after loading',
    )
    parsed = parser.parse_args(args)

    dataset = bag_to_dataset(
        parsed.bag,
        sample_dt_s=parsed.dt,
        trim_start_s=parsed.trim_start,
        trim_end_s=parsed.trim_end,
    )

    output = parsed.output
    if output is None:
        default_dir = Path(__file__).resolve().parents[1] / 'results' / 'datasets'
        output = default_dir / f'{parsed.bag.name}.npz'
    output = output.with_suffix('.npz')

    dataset.save_npz(output)
    write_dataset_yaml(output.with_suffix('.yaml'), dataset)

    duration = dataset.times_s[-1] - dataset.times_s[0] if dataset.num_samples > 1 else 0.0
    print(f'Wrote {output}')
    print(f'samples: {dataset.num_samples}, dt: {dataset.sample_dt_s:.4f} s, duration: {duration:.2f} s')


if __name__ == '__main__':
    main()
