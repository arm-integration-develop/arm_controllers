//
// Created by lsy on 23-10-30.
//
#include <dynamics_interface/dynamics_interface.h>
namespace dynamics_interface
{
bool DynamicsInterface::init(ros::NodeHandle &dynamics_nh,int arm_joints_num)
{
    if (!dynamics_nh.hasParam("urdf_filename"))
        ROS_ERROR_STREAM("NO URDF FILE PATH");
    dynamics_nh.getParam("urdf_filename",urdf_filename_);
    dynamics_nh.getParam("send_tau",send_tau_);
    a_lp_filter_ = new LowPassFilter(dynamics_nh);
    pinocchio::urdf::buildModel(urdf_filename_,model_);
    pinocchio_data_ = pinocchio::Data(model_);
    tau_.resize(arm_joints_num);
    tau_without_a_.resize(arm_joints_num);
    tau_without_a_v_.resize(arm_joints_num);
    zero_.resize(arm_joints_num);
    q_.resize(arm_joints_num);
    v_.resize(arm_joints_num);
    last_v_.resize(arm_joints_num);
    a_.resize(arm_joints_num);
    tau_error_msg_.data.resize(arm_joints_num);
    tau_msg_.data.resize(arm_joints_num);
    tau_exe_msg_.data.resize(arm_joints_num);
    tau_without_a_msg_.data.resize(arm_joints_num);
    tau_without_a_v_msg_.data.resize(arm_joints_num);
    a_msg_.data.resize(arm_joints_num);
    a_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("a", 1);
    error_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_error", 1);
    tau_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau", 1);
    tau_exe_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_exe", 1);
    tau_without_a_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_without_a", 1);
    tau_without_a_v_pub_ = dynamics_nh.advertise<std_msgs::Float64MultiArray>("tau_without_a_v", 1);
    return true;
}
void DynamicsInterface::pubDynamics()
{
    error_pub_.publish(tau_error_msg_);
    tau_pub_.publish(tau_msg_);
    tau_exe_pub_.publish(tau_exe_msg_);
    tau_without_a_pub_.publish(tau_without_a_msg_);
    tau_without_a_v_pub_.publish(tau_without_a_v_msg_);
    a_pub_.publish(a_msg_);
}
void DynamicsInterface::computerInverseDynamics(std::vector<hardware_interface::JointStateHandle> jnt_states)
{
    for (int i = 0; i < (int)tau_.size(); ++i) {
        q_(i) = jnt_states[i].getPosition();
        v_(i) = jnt_states[i].getVelocity();
        double original_a = (v_(i)-last_v_[i])/(ros::Time::now()-last_time_).toSec();
        a_lp_filter_->input(original_a);
        a_(i) = a_lp_filter_->output();
        last_v_[i] = v_(i);
        zero_[i] = 0.;
    }
    last_time_ = ros::Time::now();
    tau_ = pinocchio::rnea(model_,pinocchio_data_,q_,v_,a_);
    tau_without_a_ = pinocchio::rnea(model_,pinocchio_data_,q_,v_,zero_);
    tau_without_a_v_ = pinocchio::rnea(model_,pinocchio_data_,q_,zero_,zero_);
    for (int i = 0; i < (int)tau_.size(); ++i) {
        tau_error_msg_.data[i] = tau_(i) - jnt_states[i].getEffort();
        tau_msg_.data[i] = tau_(i);
        tau_without_a_msg_.data[i] = tau_without_a_(i);
        tau_without_a_v_msg_.data[i] = tau_without_a_v_(i);
        a_msg_.data[i] = a_(i);
    }
}
}