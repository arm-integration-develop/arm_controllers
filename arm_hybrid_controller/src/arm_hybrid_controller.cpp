//
// Created by lsy on 23-10-25.
//

#include "arm_hybrid_controller/arm_hybrid_controller.h"
namespace arm_hybrid_controller
{
bool ArmHybridController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
    const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
    change_mode_server_ = controller_nh.advertiseService("change_hybrid_mode", &ArmHybridController::changeHybridMode, this);
    // Init controller_state_publisher
    controller_state_interface_.init(controller_nh,joint_names);
    joints_interface_.init(robot_hw,controller_nh);
    ros::NodeHandle nh_dynamics(controller_nh, "dynamics");
    dynamics_interface_.init(nh_dynamics,joints_interface_.num_hw_joints_);

    // Action Service
    action_server_.reset(
            new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(controller_nh, "follow_joint_trajectory",
                             std::bind(&ArmHybridController::goalCB, this, std::placeholders::_1),
                             std::bind(&ArmHybridController::cancelCB, this, std::placeholders::_1), false));
    action_server_->start();

    last_time_ = ros::Time::now();
    return true;
}

bool ArmHybridController::changeHybridMode(controller_msgs::ChangeHybridModeRequest &req,controller_msgs::ChangeHybridModeResponse &res)
{
    mode_ = static_cast<int>(req.new_mode);
    std::string mode_string;
    switch (mode_)
    {
        case GRAVITY_COMPENSATION:
            mode_string = "GRAVITY_COMPENSATION";
            break;
        case TRAJECTORY_TEACHING:
            mode_string = "TRAJECTORY_TEACHING";
            break;
        case TRAJECTORY_TRACKING:
            mode_string = "TRAJECTORY_TRACKING";
            break;
        case HOLDING_POSITION:
        {
            mode_string = "HOLDING_POSITION";
            controller_state_interface_.initDesiredState(joints_interface_.jnt_states_);
        }
        break;
    }
    res.reply = "Now the mode is:"+ mode_string;
    ROS_INFO_STREAM(res.reply);
    return true;
}
void ArmHybridController::starting(const ros::Time& time)
{
    controller_state_interface_.initTimeData(time);
//    mode_ = GRAVITY_COMPENSATION;
    mode_ = HOLDING_POSITION;
    controller_state_interface_.initDesiredState(joints_interface_.jnt_states_);
}
void ArmHybridController::moveJoint(const ros::Time &time, const ros::Duration &period)
{
    joints_interface_.moveJoint(time,period);
}
void ArmHybridController::update(const ros::Time &time, const ros::Duration &period)
{
    controller_state_interface_.updateTimeData(time,period);
    dynamics_interface_.pubDynamics();
    controller_state_interface_.update(time,joints_interface_.jnt_states_);
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
        double position_pid_value = joints_interface_.joints_[i].position_pid_->computeCommand(controller_state_interface_.state_error_.position[i],now-last_time_);
        double cmd = position_pid_value;
        joints_interface_.joints_[i].exe_effort_ = cmd;
    }
}
}// namespace arm_hybrid_controller