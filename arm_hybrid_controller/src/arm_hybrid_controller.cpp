//
// Created by lsy on 23-10-25.
//

#include "arm_hybrid_controller/arm_hybrid_controller.h"
namespace arm_hybrid_controller
{
bool ArmHybridController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
    const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
    num_hw_joints_ = (int)joint_names.size();
    // Init controller_state_publisher
    controller_state_interface_.init(controller_nh,joint_names);
    ros::NodeHandle nh_dynamics(controller_nh, "dynamics");
    dynamics_interface_.init(nh_dynamics,num_hw_joints_);
    for (int i = 0; i < num_hw_joints_; i++)
        jnt_states_.push_back(robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(joint_names[i]));
    last_time_ = ros::Time::now();
    ros::NodeHandle nh_position_pids(controller_nh, "gains");
    ros::NodeHandle nh_joints(controller_nh, "joints");
    hardware_interface::EffortJointInterface* effort_joint_interface;
    effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    for (const auto& name : joint_names)
    {
        ros::NodeHandle nh_joint(nh_joints, name);
        ros::NodeHandle nh_pid(nh_position_pids, name);
        Joint j{ .name_ = name,
                 .position_pid_ = new control_toolbox::Pid(),
                .effort_ctrl_ = new effort_controllers::JointEffortController()};
        j.position_pid_->init(nh_pid);
        j.effort_ctrl_->init(effort_joint_interface, nh_joint);
        joints_.push_back(j);
    }
    return true;
}

void ArmHybridController::starting(const ros::Time& time)
{
    controller_state_interface_.initTimeData(time);
    mode_ = GRAVITY_COMPENSATION;
}
void ArmHybridController::moveJoint(const ros::Time &time, const ros::Duration &period)
{
    for (int i = 0; i < (int)joints_.size(); ++i) {
        joints_[i].effort_ctrl_->command_buffer_.writeFromNonRT(joints_[i].exe_effort_);
        joints_[i].effort_ctrl_->update(time, period);
    }
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
    controller_state_interface_.publishState(time);
}
void ArmHybridController::gravity_compensation()
{
    dynamics_interface_.computerInverseDynamics(jnt_states_);
    for (int i = 0; i < (int)joints_.size(); ++i) {
        joints_[i].exe_effort_ = dynamics_interface_.tau_without_a_(i);
    }
}
void ArmHybridController::holding_position(const ros::Time& now)
{
    for (int i = 0; i < (int)joints_.size(); ++i) {
        joints_[i].exe_effort_ = joints_[i].position_pid_->computeCommand(0.,now-last_time_);
    }
}
}// namespace arm_hybrid_controller