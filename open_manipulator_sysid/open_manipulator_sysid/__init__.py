"""OpenMANIPULATOR-X system identification.

Domain-aligned subpackages (see CONTEXT.md):

excitation_trajectory
    Excitation trajectory design, GA optimization, ROS runner.
excitation_recording
    Excitation recording bag → Identification dataset.
five_link_dynamics
    Five-link dynamics model (SVD BIP), reference tables.
system_identification
    System identification (BIP + Fv/Fc least squares).
model_validation
    Model validation run (BIP compare, torque/trajectory plots).
pinocchio_support
    Pinocchio model loading and shared numerics.
reference
    URDF nominal minimal-parameter reference tables.
_experimental
    Non-production experiments (isolated from main path).
"""
