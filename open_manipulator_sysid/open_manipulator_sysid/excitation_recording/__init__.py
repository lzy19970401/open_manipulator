"""Excitation recording bag → uniform identification dataset."""

from open_manipulator_sysid.excitation_recording.bag_reader import (
    EXCITATION_RECORDING_TOPICS,
    XM430_PRESENT_CURRENT_NM_MULTIPLIER,
    arm_effort_array_to_nm,
    bag_to_dataset,
    commanded_reference_from_bag,
    detect_bag_storage_id,
    interpolate_arrays_at_times,
    main,
)

__all__ = [
    'EXCITATION_RECORDING_TOPICS',
    'XM430_PRESENT_CURRENT_NM_MULTIPLIER',
    'arm_effort_array_to_nm',
    'bag_to_dataset',
    'commanded_reference_from_bag',
    'detect_bag_storage_id',
    'interpolate_arrays_at_times',
    'main',
]
