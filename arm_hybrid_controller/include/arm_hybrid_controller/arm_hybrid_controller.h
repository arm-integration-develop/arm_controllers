//
// Created by lsy on 23-10-25.
//

#pragma once

#include <pinocchio/fwd.hpp>
//must add fwd.hpp before other .h for solve the conflict between ROS and pinocchio
//https://github.com/wxmerkt/pinocchio_ros_example
#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/dynamics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
//#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_effort_controller.h>

#include <string>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>

namespace arm_hybrid_controller
{
class Joint
{
public:
    std::string name_;
    effort_controllers::JointEffortController* effort_ctrl_;
};

class ArmHybridController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
        hardware_interface::JointStateInterface>
{
public:
    ArmHybridController() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

private:
    void moveJoint(const ros::Time& time, const ros::Duration& period);
    void commandCB(const geometry_msgs::PointStampedConstPtr & msg)
    {
//        cmd_rt_buffer_.writeFromNonRT(*msg);
    }
    void computerInverseDynamics();
    void publish_info();
    pinocchio::Model model_;
    std::string urdf_filename_;
    pinocchio::Data pinocchio_data_;
    Eigen::VectorXd tau_{},tau_without_a_{},tau_without_a_v_{};
    Eigen::VectorXd q_{};
    Eigen::VectorXd v_{};
    Eigen::VectorXd last_v_{};
    Eigen::VectorXd a_{};
    Eigen::VectorXd zero_{};

    ros::Time last_time_{};
    ros::NodeHandle node_;
    ros::Publisher error_pub_,tau_pub_,tau_without_a_pub_,tau_without_a_v_pub_,tau_exe_pub_;
    std_msgs::Float64MultiArray tau_error_msg_{},tau_msg_{},tau_without_a_msg_{},tau_without_a_v_msg_{},tau_exe_msg_{};
//    realtime_tools::RealtimeBuffer<geometry_msgs::PointStamped> cmd_rt_buffer_;

    bool send_tau_ = false;
    int num_hw_joints_;
    std::vector<std::string> joint_names_{};
    std::vector<hardware_interface::JointStateHandle> jnt_states_;

    std::vector<Joint> joints_{};



};
}// namespace arm_hybrid_controller

PLUGINLIB_EXPORT_CLASS(arm_hybrid_controller::ArmHybridController, controller_interface::ControllerBase)