//#include "open_manipulator_commander/commander_template.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <open_manipulator_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using PoseCmd = open_manipulator_interfaces::msg::PoseCommand;
using namespace std::placeholders;  // for _1, _2, _3..

class Commander
{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {
            node_ = node;
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            arm_->setMaxVelocityScalingFactor(0.1);
            arm_->setMaxAccelerationScalingFactor(0.1);
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

            open_gripper_sub_ = node_->create_subscription<Bool>(
                "/command/open_gripper", 10, std::bind(&Commander::openGripperCallback, this, _1));
            
            joint_values_sub_ = node_->create_subscription<Float64MultiArray>(
                "/command/joint_values", 10, std::bind(&Commander::jointValuesCallback, this, _1));
            pose_command_sub_ = node_->create_subscription<PoseCmd>(
                "/command/pose_command", 10, std::bind(&Commander::poseCommandCallback, this, _1));
         }

        void gotoNamedTarget(const std::string &name)
        {
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(name);
            planAndExecute(arm_);
        }

        void gotoJointTarget(const std::vector<double> &joint_values)
        {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joint_values);
            planAndExecute(arm_);
        }

        void gotoPoseTarget(double x, double y, double z, 
                            double roll, double pitch, double yaw, 
                            bool cartesian_path = false)
        {
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q.normalize();
            //q = q.normalize();

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();
            target_pose.header.frame_id = "world";

            arm_->setStartStateToCurrentState();

            if (!cartesian_path) {
                arm_->setPoseTarget(target_pose);
                planAndExecute(arm_);
            } else {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                moveit_msgs::msg::RobotTrajectory trajectory;

                double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
                
                if (fraction == 1.0) {
                    arm_->execute(trajectory);
                }
            }
            
        }

        void openGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("open");
            planAndExecute(gripper_);
        }

        void closeGripper()
        {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("close");
            planAndExecute(gripper_);
        }

    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
                interface->execute(plan);
            }
        }


        void openGripperCallback(const Bool &msg)
        {
            if (msg.data) {
                openGripper();
            }
            else {
                closeGripper();
            }
        }

        void jointValuesCallback(const Float64MultiArray &msg)
        {
            auto joint_values = msg.data;
            if (joint_values.size() == 4) {
                gotoJointTarget(joint_values);
            }
        }

        void poseCommandCallback(const PoseCmd &msg)
        {
            gotoPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
        }

        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
        rclcpp::Subscription<Float64MultiArray>::SharedPtr joint_values_sub_;
        rclcpp::Subscription<PoseCmd>::SharedPtr pose_command_sub_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
