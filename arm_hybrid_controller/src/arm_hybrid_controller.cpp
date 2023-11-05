//
// Created by lsy on 23-10-25.
//

#include "arm_hybrid_controller/arm_hybrid_controller.h"
namespace arm_hybrid_controller
{
bool ArmHybridController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
    controller_nh_ = controller_nh;
    const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
    change_mode_server_ = controller_nh_.advertiseService("change_hybrid_mode", &ArmHybridController::changeHybridMode, this);
    // Init controller_state_publisher
    controller_state_interface_.init(controller_nh_,joint_names);
    joints_interface_.init(robot_hw,controller_nh_);
    ros::NodeHandle nh_dynamics(controller_nh_, "dynamics");
    dynamics_interface_.init(nh_dynamics);

    // Action Service
    action_server_.reset(
            new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(controller_nh_, "follow_joint_trajectory",
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
            trajectory_tracking(time,period);
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
void ArmHybridController::trajectory_tracking(const ros::Time& time, const ros::Duration& period) {
    // Get currently followed trajectory
    TrajectoryPtr curr_traj_ptr;
//    curr_trajectory_box_.get(curr_traj_ptr);
//    Trajectory& curr_traj = *curr_traj_ptr;

    controller_state_interface_.old_time_data_ = *(controller_state_interface_.time_data_.readFromRT());
    // Update time data
    TimeData time_data;
    time_data.time   = time;                                     // Cache current time
    time_data.period = period;                                   // Cache current control period
    time_data.uptime = controller_state_interface_.old_time_data_.uptime + period; // Update controller uptime
    controller_state_interface_.time_data_.writeFromNonRT(time_data);

    controller_state_interface_.updateDesiredStates<Trajectory>(time_data.uptime, curr_traj_ptr.get());
    int points_size = static_cast<int>(points_.size());
    if (point_current_<(points_size-1))
    {
        ROS_INFO_STREAM("now"<<time-new_gl_time_);
        ROS_INFO_STREAM("points"<<points_[point_current_].time_from_start);
    }
    if (time-new_gl_time_ > points_[point_current_].time_from_start && point_current_<(points_size-1))
    {
        for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
            controller_state_interface_.desired_state_.position[i] = points_[point_current_].positions[i];
            controller_state_interface_.desired_state_.velocity[i] = points_[point_current_].velocities[i];
            controller_state_interface_.desired_state_.acceleration[i] = points_[point_current_].accelerations[i];
        }
        point_current_++;
        ROS_INFO_STREAM(point_current_);
    }
    for (int i = 0; i < joints_interface_.num_hw_joints_; ++i) {
        double position_pid_value = joints_interface_.joints_[i].position_pid_->computeCommand(controller_state_interface_.state_error_.position[i],time-last_time_);
        double cmd = position_pid_value;
        joints_interface_.joints_[i].exe_effort_ = cmd;
    }
}
void ArmHybridController::preemptActiveGoal()
{
    RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

    // Cancels the currently active goal
    if (current_active_goal)
    {
        // Marks the current goal as canceled
        rt_active_goal_.reset();
        current_active_goal->gh_.setCanceled();
    }
}

void ArmHybridController::goalCB(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
    mode_ = TRAJECTORY_TRACKING;
//        Use for test trajectory without segment
    point_current_ = 0;
    points_ = gh.getGoal()->trajectory.points;
    new_gl_time_ = ros::Time::now();
    ROS_INFO_STREAM(mode_);
//        Use for test trajectory without segment
    RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
    std::string error_string;
    const bool update_ok = true;
//    const bool update_ok = updateTrajectoryCommand(internal::share_member(gh.getGoal(), gh.getGoal()->trajectory),
//                                                   rt_goal,
//                                                   &error_string);
    rt_goal->preallocated_feedback_->joint_names = joints_interface_.joint_names_;
    if (update_ok)
    {
        // Accept new goal
        preemptActiveGoal();
        gh.setAccepted();
        rt_active_goal_ = rt_goal;
        // Setup goal status checking timer
        goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,&RealtimeGoalHandle::runNonRealtime,rt_goal);
        goal_handle_timer_.start();
    }
    else
    {
        // Reject invalid goal
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        result.error_string = error_string;
        gh.setRejected(result);
    }
}
}// namespace arm_hybrid_controller