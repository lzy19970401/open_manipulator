"""System identification — joint LS regression (BIP + Fv/Fc)."""

from open_manipulator_sysid.system_identification.friction import (
    FRICTION_PARAM_NAMES,
    NUM_FRICTION_PARAMS,
    CoulombViscousFriction,
)

__all__ = [
    'CoulombViscousFriction',
    'FRICTION_PARAM_NAMES',
    'NUM_FRICTION_PARAMS',
]
