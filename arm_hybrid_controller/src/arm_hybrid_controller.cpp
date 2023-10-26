//
// Created by lsy on 23-10-25.
//

#include "arm_hybrid_controller/arm_hybrid_controller.h"
namespace arm_hybrid_controller
{
bool ArmHybridController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
    if (!controller_nh.hasParam("urdf_filename"))
        ROS_ERROR_STREAM("NO URDF FILE PATH");
    controller_nh.getParam("urdf_filename",urdf_filename_);
    controller_nh.getParam("send_tau",send_tau_);
    ROS_INFO_STREAM(send_tau_);
    pinocchio::urdf::buildModel(urdf_filename_,model_);
    pinocchio_data_ = pinocchio::Data(model_);
    const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
    num_hw_joints_ = (int)joint_names.size();
    for (int i = 0; i < num_hw_joints_; i++)
        jnt_states_.push_back(robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(joint_names[i]));
    tau_.resize(num_hw_joints_);
    tau_without_a_.resize(num_hw_joints_);
    tau_without_a_v_.resize(num_hw_joints_);
    zero_.resize(num_hw_joints_);
    q_.resize(num_hw_joints_);
    v_.resize(num_hw_joints_);
    last_v_.resize(num_hw_joints_);
    a_.resize(num_hw_joints_);
    tau_error_msg_.data.resize(num_hw_joints_);
    tau_msg_.data.resize(num_hw_joints_);
    tau_exe_msg_.data.resize(num_hw_joints_);
    tau_without_a_msg_.data.resize(num_hw_joints_);
    tau_without_a_v_msg_.data.resize(num_hw_joints_);
    error_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tau_error", 1);
    tau_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tau", 1);
    tau_exe_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tau_exe", 1);
    tau_without_a_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tau_without_a", 1);
    tau_without_a_v_pub_ = node_.advertise<std_msgs::Float64MultiArray>("tau_without_a_v", 1);

    hardware_interface::EffortJointInterface* effort_joint_interface;
    effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    for (const auto& name : joint_names)
    {
        Joint j{ .name_ = name,
                .effort_ctrl_ = new effort_controllers::JointEffortController()};
        ros::NodeHandle nh_joint(controller_nh, name);
        j.effort_ctrl_->init(effort_joint_interface, nh_joint);
        joints_.push_back(j);
    }
    return true;
}
void ArmHybridController::moveJoint(const ros::Time &time, const ros::Duration &period)
{
    for (int i = 0; i < (int)joints_.size(); ++i) {
        joints_[i].effort_ctrl_->command_buffer_.writeFromNonRT(tau_without_a_v_[i]);
        tau_exe_msg_.data[i] = *joints_[i].effort_ctrl_->command_buffer_.readFromRT();
        joints_[i].effort_ctrl_->update(time,period);
    }
}
void ArmHybridController::update(const ros::Time &time, const ros::Duration &period)
{
    computerInverseDynamics();
//    if(send_tau_)
        moveJoint(time,period);
    publish_info();
}
void ArmHybridController::computerInverseDynamics()
{
    for (int i = 0; i < num_hw_joints_; ++i) {
        q_(i) = jnt_states_[i].getPosition();
        v_(i) = jnt_states_[i].getVelocity();
        a_(i) = (jnt_states_[i].getVelocity()-last_v_[i])/(ros::Time::now().toSec()-last_time_.toSec());
        last_v_[i] = v_(i);
        zero_[i] = 0.;
    }
    last_time_ = ros::Time::now();
    tau_ = pinocchio::rnea(model_,pinocchio_data_,q_,v_,a_);
    tau_without_a_ = pinocchio::rnea(model_,pinocchio_data_,q_,v_,zero_);
    tau_without_a_v_ = pinocchio::rnea(model_,pinocchio_data_,q_,zero_,zero_);
    for (int i = 0; i < num_hw_joints_; ++i) {
        tau_error_msg_.data[i] = tau_(i) - jnt_states_[i].getEffort();
        tau_msg_.data[i] = tau_(i);
        tau_without_a_msg_.data[i] = tau_without_a_(i);
        tau_without_a_v_msg_.data[i] = tau_without_a_v_(i);
    }
}
void ArmHybridController::publish_info()
{
    error_pub_.publish(tau_error_msg_);
    tau_pub_.publish(tau_msg_);
    tau_exe_pub_.publish(tau_exe_msg_);
    tau_without_a_pub_.publish(tau_without_a_msg_);
    tau_without_a_v_pub_.publish(tau_without_a_v_msg_);
}
}// namespace arm_hybrid_controller