"""Rosbag2 → :class:`IdentificationDataset` (read bags, resample, validate)."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS
from open_manipulator_sysid.identification_dataset import (
    IdentificationDataset,
    filtered_acceleration,
    resample_uniform,
    validate_identification_dataset,
)
# XM430-W350-R/T — same factor as open_manipulator_x_current.ros2_control.xacro.
XM430_PRESENT_CURRENT_NM_MULTIPLIER = 0.00479627
# Stall torque ~4.1 N·m; magnitudes above this are almost certainly Dynamixel LSB.
MAX_REASONABLE_ARM_TORQUE_NM = 5.0


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


def bag_to_dataset(
    bag_path: Path | str,
    *,
    sample_dt_s: float = 0.01,
    smooth_window: int = 11,
    trim_start_s: Optional[float] = None,
    trim_end_s: Optional[float] = None,
) -> IdentificationDataset:
    """Convert rosbag2 joint_states into a uniform identification dataset."""
    times_s, raw = joint_states_from_bag(bag_path)

    if trim_start_s is not None or trim_end_s is not None:
        start_time = times_s[0] + (trim_start_s or 0.0)
        end_time = times_s[-1] - (trim_end_s or 0.0)
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
