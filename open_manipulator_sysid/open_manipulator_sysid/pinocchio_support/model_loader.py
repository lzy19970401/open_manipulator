"""Pinocchio model loading utilities shared by sysid modules."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover
    pin = None
    _PINOCCHIO_IMPORT_ERROR = exc
else:
    _PINOCCHIO_IMPORT_ERROR = None

INERTIA_PARAM_NAMES = (
    'mass',
    'mc_x',
    'mc_y',
    'mc_z',
    'Ixx',
    'Ixy',
    'Iyy',
    'Ixz',
    'Iyz',
    'Izz',
)
SVD_RANK_RTOL = 1e-6
DEFAULT_RESULTS_DIR = Path('/workspace/sysid_results')


def require_pinocchio() -> None:
    if pin is None:
        raise ImportError(
            'Pinocchio is required for offline sysid. '
            'Install ros-jazzy-pinocchio inside the Docker container.'
        ) from _PINOCCHIO_IMPORT_ERROR


def _description_share_subpath(*parts: str) -> Path:
    source_path = Path(__file__).resolve().parents[2] / 'open_manipulator_description'
    candidate = source_path.joinpath(*parts)
    if candidate.is_file():
        return candidate

    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('open_manipulator_description'))
        installed = share.joinpath(*parts)
        if installed.is_file():
            return installed
    except Exception:
        pass

    raise FileNotFoundError(
        f'open_manipulator_description file not found: {" / ".join(parts)}'
    )


def default_xacro_path() -> Path:
    return _description_share_subpath(
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf.xacro',
    )


def default_urdf_path() -> Path:
    return _description_share_subpath(
        'urdf',
        'open_manipulator_x',
        'open_manipulator_x.urdf',
    )


def default_xacro_mappings() -> Dict[str, str]:
    return {
        'prefix': '',
        'use_sim': 'false',
        'use_mock_hardware': 'false',
        'mock_sensor_commands': 'false',
        'port_name': '/dev/ttyUSB0',
        'ros2_control_type': 'open_manipulator_x_position',
    }


def expand_xacro(
    xacro_path: Path | str,
    mappings: Optional[Mapping[str, str]] = None,
) -> str:
    import xacro

    resolved_mappings = dict(default_xacro_mappings())
    if mappings:
        resolved_mappings.update(mappings)
    document = xacro.process_file(str(xacro_path), mappings=resolved_mappings)
    return document.toprettyxml(indent='  ')


def format_model_source(
    model_path: Path,
    *,
    xacro_mappings: Optional[Mapping[str, str]] = None,
) -> str:
    if xacro_mappings is None:
        return str(model_path)
    pairs = ' '.join(f'{key}:={value}' for key, value in sorted(xacro_mappings.items()))
    return f'{model_path} ({pairs})'


def default_results_dir() -> Path:
    DEFAULT_RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    return DEFAULT_RESULTS_DIR


def load_model(
    model_path: Path | str,
    *,
    xacro_mappings: Optional[Mapping[str, str]] = None,
):
    require_pinocchio()
    path = Path(model_path)
    if path.suffix == '.xacro' or xacro_mappings is not None:
        urdf_xml = expand_xacro(path, xacro_mappings)
        model = pin.buildModelFromXML(urdf_xml)
    else:
        model = pin.buildModelFromUrdf(str(path))
    data = model.createData()
    return model, data


def load_default_model():
    return load_model(
        default_xacro_path(),
        xacro_mappings=default_xacro_mappings(),
    )


def resolve_model_load_args(
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
) -> Tuple[Path, Optional[Dict[str, str]], str]:
    if urdf_path is not None and xacro_path is not None:
        raise ValueError('Provide only one of --urdf or --xacro')

    if urdf_path is not None:
        path = Path(urdf_path)
        return path, None, format_model_source(path)

    path = Path(xacro_path) if xacro_path else default_xacro_path()
    resolved_mappings = dict(default_xacro_mappings())
    if xacro_mappings:
        resolved_mappings.update(xacro_mappings)
    return path, resolved_mappings, format_model_source(path, xacro_mappings=resolved_mappings)


def arm_velocity_indices(model) -> List[int]:
    return [model.joints[model.getJointId(joint)].idx_v for joint in ARM_JOINTS]


def nominal_dynamic_parameters(model) -> np.ndarray:
    return np.concatenate(
        [
            model.inertias[body_index].toDynamicParameters()
            for body_index in range(1, model.nbodies)
        ]
    )


def body_names(model) -> List[str]:
    return [model.names[body_index] for body_index in range(1, model.nbodies)]


def stack_state_from_sample(
    model,
    sample,
    arm_indices: Sequence[int],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = pin.neutral(model)
    velocity = np.zeros(model.nv)
    acceleration = np.zeros(model.nv)
    for joint_index, joint_name in enumerate(ARM_JOINTS):
        idx = arm_indices[joint_index]
        q[idx] = sample.position[joint_name]
        velocity[idx] = sample.velocity[joint_name]
        acceleration[idx] = sample.acceleration[joint_name]
    return q, velocity, acceleration


def effective_rank_and_condition(regressor: np.ndarray) -> Tuple[int, float]:
    singular_values = np.linalg.svd(regressor, compute_uv=False)
    if singular_values.size == 0:
        return 0, float('nan')
    threshold = SVD_RANK_RTOL * singular_values[0]
    rank = int(np.count_nonzero(singular_values > threshold))
    if rank == 0:
        return 0, float('inf')
    condition_number = float(singular_values[0] / singular_values[rank - 1])
    return rank, condition_number


def inertia_entries(model, values: np.ndarray) -> List[Dict[str, object]]:
    entries: List[Dict[str, object]] = []
    for body_index, body_name in enumerate(body_names(model), start=0):
        offset = body_index * len(INERTIA_PARAM_NAMES)
        chunk = values[offset:offset + len(INERTIA_PARAM_NAMES)]
        entries.append(
            {
                'body': body_name,
                'parameters': {
                    name: float(value)
                    for name, value in zip(INERTIA_PARAM_NAMES, chunk)
                },
            }
        )
    return entries


def _parse_xacro_mapping_arg(value: str) -> Tuple[str, str]:
    if ':=' not in value:
        raise argparse.ArgumentTypeError(
            f'Xacro mapping must be KEY:=VALUE, got {value!r}'
        )
    key, mapped_value = value.split(':=', 1)
    key = key.strip()
    if not key:
        raise argparse.ArgumentTypeError(
            f'Xacro mapping key must be non-empty, got {value!r}'
        )
    return key, mapped_value.strip()


def _add_model_source_arguments(parser: argparse.ArgumentParser) -> None:
    model_group = parser.add_argument_group('robot model source')
    model_group.add_argument('--urdf', type=Path, default=None)
    model_group.add_argument('--xacro', type=Path, default=None)
    model_group.add_argument(
        '--xacro-mapping',
        action='append',
        default=[],
        metavar='KEY:=VALUE',
        type=_parse_xacro_mapping_arg,
    )


def _model_source_kwargs_from_parsed(parsed: argparse.Namespace) -> Dict[str, object]:
    return {
        'urdf_path': parsed.urdf,
        'xacro_path': parsed.xacro,
        'xacro_mappings': dict(parsed.xacro_mapping) if parsed.xacro_mapping else None,
    }


def main(args: Optional[Sequence[str]] = None) -> None:
    from open_manipulator_sysid.system_identification.identify import main as identify_main

    identify_main(args)


def main_bag(args: Optional[Sequence[str]] = None) -> None:
    from open_manipulator_sysid.system_identification.identify import main_bag as identify_bag_main

    identify_bag_main(args)


if __name__ == '__main__':
    main()
