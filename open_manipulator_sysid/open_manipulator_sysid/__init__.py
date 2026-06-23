"""OpenMANIPULATOR-X system identification utilities.

Package layout (one module per seam):

excitation_trajectory
    Fourier trajectory, safety limits, quintic approach — config-driven math only.
excitation_runner
    ROS node: send FollowJointTrajectory goal to arm_controller.
bag_to_dataset
    Rosbag2 ``/joint_states`` → uniform dataset (includes effort LSB → N·m).
identification_dataset
    ``IdentificationDataset`` container, resampling, validation.
regress
    Offline friction least squares (fixed URDF inertia).
"""
