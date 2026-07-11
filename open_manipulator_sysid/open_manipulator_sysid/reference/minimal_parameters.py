"""Minimal dynamics parameter reference: five-link BIP + Coulomb/viscous friction."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from open_manipulator_sysid.system_identification.friction import (
    CoulombViscousFriction,
    FRICTION_PARAM_NAMES,
    format_friction_equation,
    reference_friction_vector,
)
from open_manipulator_sysid.excitation_trajectory import ARM_JOINTS, ExcitationTrajectory, default_config_path
from open_manipulator_sysid.five_link_dynamics.base_parameters import (
    FiveLinkBaseParameterSet,
)
from open_manipulator_sysid.five_link_dynamics.reference_tables import derive_five_link_reference_table
from open_manipulator_sysid.pinocchio_support.model_loader import (
    _add_model_source_arguments,
    _model_source_kwargs_from_parsed,
    load_model,
    resolve_model_load_args,
)


@dataclass(frozen=True)
class MinimalParameterReferenceRow:
    index: int
    mathematical_form: str
    physical_meaning: str
    unit: str
    reference_value: float


@dataclass(frozen=True)
class MinimalParameterReference:
    base_set: FiveLinkBaseParameterSet
    base_reference_values: np.ndarray
    friction_reference: CoulombViscousFriction
    friction_reference_vector: np.ndarray

    @property
    def num_base(self) -> int:
        return self.base_set.num_base

    @property
    def num_friction(self) -> int:
        return len(self.friction_reference_vector)

    @property
    def num_total(self) -> int:
        return self.num_base + self.num_friction


def derive_minimal_parameter_reference(
    *,
    urdf_path: Optional[Path | str] = None,
    xacro_path: Optional[Path | str] = None,
    xacro_mappings: Optional[Mapping[str, str]] = None,
    excitation_config: Path | str,
) -> Tuple[object, MinimalParameterReference]:
    model, base_set, ref_values = derive_five_link_reference_table(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        xacro_mappings=xacro_mappings,
        excitation_config=excitation_config,
    )
    with Path(excitation_config).open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle) or {}
    friction = CoulombViscousFriction.from_config(config)
    return model, MinimalParameterReference(
        base_set=base_set,
        base_reference_values=ref_values,
        friction_reference=friction,
        friction_reference_vector=reference_friction_vector(friction),
    )


def build_reference_rows(reference: MinimalParameterReference) -> list[MinimalParameterReferenceRow]:
    rows: list[MinimalParameterReferenceRow] = []
    expressions = reference.base_set.symbolic_expressions()
    units = reference.base_set.units()
    for index, (math_form, unit, value) in enumerate(
        zip(expressions, units, reference.base_reference_values),
        start=1,
    ):
        rows.append(
            MinimalParameterReferenceRow(
                index=index,
                mathematical_form=math_form,
                physical_meaning=f'BIP combo {index}',
                unit=unit,
                reference_value=float(value),
            )
        )
    base_count = len(rows)
    for joint_index, joint in enumerate(ARM_JOINTS):
        for param_index, param_name in enumerate(FRICTION_PARAM_NAMES):
            offset = base_count + joint_index * len(FRICTION_PARAM_NAMES) + param_index + 1
            if param_name == 'Fv':
                math_form = f'{param_name}_{joint_index + 1}·q̇_{joint_index + 1}'
                unit = 'N·m·s/rad'
            else:
                math_form = f'{param_name}_{joint_index + 1}·sign(q̇_{joint_index + 1})'
                unit = 'N·m'
            rows.append(
                MinimalParameterReferenceRow(
                    index=offset,
                    mathematical_form=math_form,
                    physical_meaning=f'{joint} {param_name} ({format_friction_equation(joint)})',
                    unit=unit,
                    reference_value=float(
                        reference.friction_reference_vector[
                            joint_index * len(FRICTION_PARAM_NAMES) + param_index
                        ]
                    ),
                )
            )
    return rows


def format_reference_markdown(reference: MinimalParameterReference) -> str:
    rows = build_reference_rows(reference)
    lines = [
        '## Minimal parameter set (link1–link5, gripper excluded)',
        '',
        'Dynamics: τ = Y_BIP(q,q̇,q̈)·π_BIP + τ_friction(q̇)',
        '',
        'Friction: τ_f,j = Fv_j·q̇_j + Fc_j·sign(q̇_j)',
        '',
        'Note: **link1** inertia is on the fixed base (Pinocchio universe) and does not appear',
        'as a regressor column; BIP combinations below use **link2–link5** (40 standard columns).',
        '',
        f'| i | Mathematical form | Physical meaning | Unit | θ (reference) |',
        f'|---|-------------------|------------------|------|---------------|',
    ]
    for row in rows:
        lines.append(
            f'| {row.index} | {row.mathematical_form} | {row.physical_meaning} | '
            f'{row.unit} | {row.reference_value:.6g} |'
        )
    lines.append('')
    lines.append(
        f'**Counts:** {reference.num_base} BIP + {reference.num_friction} friction '
        f'= {reference.num_total} identifiable parameters (GA target rank).'
    )
    return '\n'.join(lines)


def write_reference_yaml(
    path: Path | str,
    *,
    model_source: str,
    excitation_config: Path | str,
    reference: MinimalParameterReference,
) -> None:
    rows = build_reference_rows(reference)
    payload = {
        'description': (
            'Minimal dynamics parameter reference for OMX sysid: five-link symbolic BIP '
            '(gripper inertia excluded, mass on link5) plus Coulomb/viscous friction.'
        ),
        'model_source': model_source,
        'excitation_config': str(excitation_config),
        'dynamics_equation': 'tau = Y_BIP(q,qd,qdd) @ pi_BIP + tau_friction(qd)',
        'friction_equation': 'tau_f,j = Fv_j * qd_j + Fc_j * sign(qd_j)',
        'five_link_base_parameters': {
            **reference.base_set.to_dict(),
            'reference_values': [float(value) for value in reference.base_reference_values],
        },
        'friction': {
            'Fv': [float(value) for value in reference.friction_reference.fv],
            'Fc': [float(value) for value in reference.friction_reference.fc],
            'reference_vector': [float(value) for value in reference.friction_reference_vector],
        },
        'reference_table': [
            {
                'index': row.index,
                'mathematical_form': row.mathematical_form,
                'physical_meaning': row.physical_meaning,
                'unit': row.unit,
                'reference_value': row.reference_value,
            }
            for row in rows
        ],
        'parameter_counts': {
            'bip': reference.num_base,
            'friction': reference.num_friction,
            'total': reference.num_total,
        },
    }
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, default_flow_style=False)


def main(args: Optional[Sequence[str]] = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description=(
            'Export minimal dynamics parameter reference from URDF: five-link BIP '
            'table plus Fv/Fc friction (gripper excluded).'
        )
    )
    _add_model_source_arguments(parser)
    parser.add_argument('--config', type=Path, default=None)
    parser.add_argument(
        '--output-yaml',
        type=Path,
        default=Path('/workspace/sysid_results/minimal_parameter_reference.yaml'),
    )
    parsed = parser.parse_args(args)

    config_path = parsed.config or default_config_path()
    _, _, model_source = resolve_model_load_args(**_model_source_kwargs_from_parsed(parsed))
    _, reference = derive_minimal_parameter_reference(
        **_model_source_kwargs_from_parsed(parsed),
        excitation_config=config_path,
    )
    print(format_reference_markdown(reference))
    write_reference_yaml(
        parsed.output_yaml,
        model_source=str(model_source),
        excitation_config=config_path,
        reference=reference,
    )
    print(f'\nWrote reference yaml: {parsed.output_yaml}')


if __name__ == '__main__':
    main()
