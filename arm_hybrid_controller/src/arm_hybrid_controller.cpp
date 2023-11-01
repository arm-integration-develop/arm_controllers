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
    ros::NodeHandle nh_dynamics(controller_nh, "dynamics");
    dynamics_interface_.init(nh_dynamics,num_hw_joints_);
    for (int i = 0; i < num_hw_joints_; i++)
        jnt_states_.push_back(robot_hw->get<hardware_interface::JointStateInterface>()->getHandle(joint_names[i]));

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
//        joints_[i].effort_ctrl_->command_buffer_.writeFromNonRT(tau_without_a_v_[i]);
        joints_[i].effort_ctrl_->command_buffer_.writeFromNonRT(dynamics_interface_.tau_without_a_(i));
        joints_[i].effort_ctrl_->update(time,period);
    }
}
void ArmHybridController::update(const ros::Time &time, const ros::Duration &period)
{
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
            holding_position();
            break;
    }
    if(dynamics_interface_.send_tau_)
        moveJoint(time,period);
    dynamics_interface_.pubDynamics();
}
void ArmHybridController::gravity_compensation()
{
    dynamics_interface_.computerInverseDynamics(jnt_states_);
}
}// namespace arm_hybrid_controller