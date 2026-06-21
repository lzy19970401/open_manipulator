#!/usr/bin/env python3
"""
Standalone teaching example — NOT part of open_manipulator ROS packages.
Pinocchio gravity compensation g(q) for OpenMANIPULATOR-X arm.

Prerequisites:
  sudo apt install ros-${ROS_DISTRO}-pinocchio   # or pip install pin
  export URDF_PATH=/path/to/open_manipulator_x.urdf

Usage:
  python3 teaching-examples/pinocchio_gravity_comp.py
  python3 teaching-examples/pinocchio_gravity_comp.py /path/to/model.urdf 0 -1 0.7 0.3
"""

import sys
import os

import pinocchio as pin
import numpy as np


def main():
    urdf = os.environ.get(
        "URDF_PATH",
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "open_manipulator_description",
            "urdf",
            "open_manipulator_x",
            "open_manipulator_x.urdf",
        ),
    )
    urdf = os.path.abspath(urdf)

    if len(sys.argv) > 1:
        urdf = sys.argv[1]

    if not os.path.isfile(urdf):
        print(f"URDF not found: {urdf}")
        print("Set URDF_PATH or pass path as argv[1]")
        sys.exit(1)

    # Fixed base: do not add a free-flyer root joint
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()

    print(f"Model: nq={model.nq}, nv={model.nv}")
    print(f"Joint names: {[model.names[i] for i in range(1, model.njoints)]}")

    # Default q: SRDF home pose for arm, or from CLI
    if len(sys.argv) >= 6:
        q_arm = np.array([float(sys.argv[i]) for i in range(2, 6)])
    else:
        q_arm = np.array([0.0, -1.0, 0.7, 0.3])  # home

    q = pin.neutral(model)
    # Map first 4 velocity DoF (arm) — adjust if your URDF model layout differs
    nv_arm = min(4, model.nv)
    q[:nv_arm] = q_arm[:nv_arm]

    # Gravity vector (default is (0,0,-9.81) in pinocchio)
    print(f"model.gravity.linear = {model.gravity.linear.T}")

    g = pin.computeGeneralizedGravity(model, data, q)
    tau_rnea = pin.rnea(
        model, data, q, np.zeros(model.nv), np.zeros(model.nv)
    )

    print(f"\nq (first {nv_arm} coords) = {q[:nv_arm].T}")
    print(f"g(q) = {g[:nv_arm].T}")
    print(f"rnea(q,0,0) = {tau_rnea[:nv_arm].T}")
    print(f"max |g - rnea| = {np.max(np.abs(g - tau_rnea)):.2e}")

    print("\n→ Command +g(q) on effort interface to compensate gravity (sign may need")
    print("  calibration vs Dynamixel / KDL convention on real hardware).")


if __name__ == "__main__":
    main()
