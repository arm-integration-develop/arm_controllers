//
// Created by lsy on 23-10-30.
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
#include <hardware_interface/joint_state_interface.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tools/lp_filter.h>

namespace dynamics_interface
{
class DynamicsInterface
{
public:
    bool init(ros::NodeHandle &controller_nh,int arm_joints_num);
    void pubDynamics();
    void computerInverseDynamics(std::vector<hardware_interface::JointStateHandle> jnt_states);
    Eigen::VectorXd tau_{},tau_without_a_{},tau_without_a_v_{};
    Eigen::VectorXd q_{},v_{},last_v_{},a_{},zero_{};
    bool send_tau_ = false;

private:
    pinocchio::Model model_;
    std::string urdf_filename_;
    pinocchio::Data pinocchio_data_;

    ros::Time last_time_{};
    ros::NodeHandle node_;
    ros::Publisher error_pub_,tau_pub_,tau_without_a_pub_,tau_without_a_v_pub_,tau_exe_pub_,a_pub_;
    std_msgs::Float64MultiArray tau_error_msg_{},tau_msg_{},tau_without_a_msg_{},tau_without_a_v_msg_{},tau_exe_msg_{},a_msg_{};

    int num_hw_joints_;
    LowPassFilter* a_lp_filter_;
};
}