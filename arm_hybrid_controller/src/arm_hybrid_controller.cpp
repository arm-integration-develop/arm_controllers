//
// Created by lsy on 23-10-25.
//

#include "arm_hybrid_controller/arm_hybrid_controller.h"
namespace arm_hybrid_controller
{
bool ArmHybridController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
    const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();

    // Init controller_state_publisher
    controller_state_interface_.init(controller_nh,joint_names);
    joints_interface_.init(robot_hw,controller_nh);
    ros::NodeHandle nh_dynamics(controller_nh, "dynamics");
    dynamics_interface_.init(nh_dynamics,joints_interface_.num_hw_joints_);

    last_time_ = ros::Time::now();
    return true;
}

void ArmHybridController::starting(const ros::Time& time)
{
    controller_state_interface_.initTimeData(time);
    mode_ = GRAVITY_COMPENSATION;
    controller_state_interface_.initDesiredState(joints_interface_.jnt_states_);
}
void ArmHybridController::moveJoint(const ros::Time &time, const ros::Duration &period)
{
    joints_interface_.moveJoint(time,period);
}
void ArmHybridController::update(const ros::Time &time, const ros::Duration &period)
{
    controller_state_interface_.updateTimeData(time,period);
    switch (mode_) 
    {
        case GRAVITY_COMPENSATION:
            gravity_compensation();
            break;
        case TRAJECTORY_TEACHING:
            trajectory_teaching();
            break;
        case TRAJECTORY_TRACKING:
            trajectory_tracking();
            break;
        case HOLDING_POSITION:
            holding_position(time);
            break;
    }
    last_time_ = time;
    if(dynamics_interface_.send_tau_)
        moveJoint(time,period);
    dynamics_interface_.pubDynamics();
    controller_state_interface_.update(time,joints_interface_.jnt_states_);
}
void ArmHybridController::gravity_compensation()
{
    dynamics_interface_.computerInverseDynamics(joints_interface_.jnt_states_);
    for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
        joints_interface_.joints_[i].exe_effort_ = dynamics_interface_.tau_without_a_(i);
    }
}
void ArmHybridController::holding_position(const ros::Time& now)
{
    for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
        joints_interface_.joints_[i].exe_effort_ = joints_interface_.joints_[i].position_pid_->computeCommand(0.,now-last_time_);
    }
}
}// namespace arm_hybrid_controller