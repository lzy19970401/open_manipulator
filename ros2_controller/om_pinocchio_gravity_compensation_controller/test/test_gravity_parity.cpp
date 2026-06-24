// Copyright 2026 OpenMANIPULATOR contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/joint.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace
{

constexpr double kParityToleranceNm = 1e-2;
constexpr double kGripperOpenM = 0.019;

const std::vector<std::string> kArmJoints = {
  "joint1", "joint2", "joint3", "joint4"
};

std::string default_urdf_path()
{
  const auto share = ament_index_cpp::get_package_share_directory("open_manipulator_description");
  return share + "/urdf/open_manipulator_x/open_manipulator_x.urdf";
}

pinocchio::Model load_pinocchio_model(const std::string & urdf_path)
{
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  return model;
}

KDL::Tree load_kdl_tree(const std::string & urdf_path)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromFile(urdf_path, tree)) {
    throw std::runtime_error("Failed to parse URDF into KDL tree: " + urdf_path);
  }
  return tree;
}

Eigen::VectorXd make_pinocchio_configuration(
  const pinocchio::Model & model,
  const std::vector<double> & arm_positions,
  double gripper_position)
{
  Eigen::VectorXd q = pinocchio::neutral(model);
  for (size_t i = 0; i < kArmJoints.size(); ++i) {
    const pinocchio::JointIndex joint_id = model.getJointId(kArmJoints[i]);
    q[model.joints[joint_id].idx_q()] = arm_positions[i];
  }
  for (const auto & gripper_joint : {"gripper_left_joint", "gripper_right_joint"}) {
    if (model.existJointName(gripper_joint)) {
      const pinocchio::JointIndex joint_id = model.getJointId(gripper_joint);
      q[model.joints[joint_id].idx_q()] = gripper_position;
    }
  }
  return q;
}

KDL::JntArray make_kdl_configuration(
  const KDL::Tree & tree,
  const std::map<std::string, double> & joint_positions)
{
  const unsigned int n_joints = tree.getNrOfJoints();
  KDL::JntArray q(n_joints);
  for (unsigned int i = 0; i < n_joints; ++i) {
    q(i) = 0.0;
  }

  for (const auto & entry : tree.getSegments()) {
    const KDL::Joint & joint = entry.second.segment.getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    const auto it = joint_positions.find(joint.getName());
    if (it != joint_positions.end()) {
      q(entry.second.q_nr) = it->second;
    }
  }
  return q;
}

std::vector<double> pinocchio_arm_gravity(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const Eigen::VectorXd & q)
{
  pinocchio::computeGeneralizedGravity(model, data, q);
  std::vector<double> gravity(kArmJoints.size(), 0.0);
  for (size_t i = 0; i < kArmJoints.size(); ++i) {
    const pinocchio::JointIndex joint_id = model.getJointId(kArmJoints[i]);
    gravity[i] = data.g[model.joints[joint_id].idx_v()];
  }
  return gravity;
}

std::vector<double> kdl_arm_gravity(
  const KDL::Tree & tree,
  const KDL::JntArray & q)
{
  KDL::TreeIdSolver_RNE id_solver(tree, KDL::Vector(0.0, 0.0, -9.81));
  KDL::JntArray qdot(tree.getNrOfJoints());
  KDL::JntArray qddot(tree.getNrOfJoints());
  KDL::JntArray torques(tree.getNrOfJoints());
  KDL::WrenchMap f_ext;

  for (unsigned int i = 0; i < tree.getNrOfJoints(); ++i) {
    qdot(i) = 0.0;
    qddot(i) = 0.0;
  }

  id_solver.CartToJnt(q, qdot, qddot, f_ext, torques);

  std::map<std::string, double> gravity_by_name;
  for (const auto & entry : tree.getSegments()) {
    const KDL::Joint & joint = entry.second.segment.getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;
    }
    gravity_by_name[joint.getName()] = torques(entry.second.q_nr);
  }

  std::vector<double> gravity(kArmJoints.size(), 0.0);
  for (size_t i = 0; i < kArmJoints.size(); ++i) {
    gravity[i] = gravity_by_name.at(kArmJoints[i]);
  }
  return gravity;
}

void expect_pose_parity(
  const std::string & pose_name,
  const pinocchio::Model & model,
  pinocchio::Data & data,
  const KDL::Tree & tree,
  const std::vector<double> & arm_positions)
{
  const Eigen::VectorXd q = make_pinocchio_configuration(model, arm_positions, kGripperOpenM);
  const auto g_pin = pinocchio_arm_gravity(model, data, q);

  std::map<std::string, double> joint_positions;
  for (size_t i = 0; i < kArmJoints.size(); ++i) {
    joint_positions[kArmJoints[i]] = arm_positions[i];
  }
  joint_positions["gripper_left_joint"] = kGripperOpenM;
  joint_positions["gripper_right_joint"] = kGripperOpenM;
  const KDL::JntArray q_kdl = make_kdl_configuration(tree, joint_positions);
  const auto g_kdl = kdl_arm_gravity(tree, q_kdl);

  for (size_t i = 0; i < kArmJoints.size(); ++i) {
    SCOPED_TRACE(pose_name + " " + kArmJoints[i]);
    EXPECT_NEAR(g_pin[i], g_kdl[i], kParityToleranceNm)
      << "Pinocchio=" << g_pin[i] << " KDL=" << g_kdl[i];
  }
}

}  // namespace

TEST(PinocchioKdlGravityParity, NamedPosesWithinTolerance)
{
  const std::string urdf_path = default_urdf_path();
  const pinocchio::Model model = load_pinocchio_model(urdf_path);
  pinocchio::Data data(model);
  const KDL::Tree tree = load_kdl_tree(urdf_path);

  expect_pose_parity("init", model, data, tree, {0.0, 0.0, 0.0, 0.0});
  expect_pose_parity("home", model, data, tree, {0.0, -1.0, 0.7, 0.3});
  expect_pose_parity("ready", model, data, tree, {0.0, -1.0, 1.0, 0.0});
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
