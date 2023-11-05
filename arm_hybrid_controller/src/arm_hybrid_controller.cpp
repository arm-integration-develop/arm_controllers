//
// Created by lsy on 23-10-25.
//

#include "arm_hybrid_controller/arm_hybrid_controller.h"
namespace arm_hybrid_controller
{
bool ArmHybridController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Action status checking update rate
  double action_monitor_rate = 20.0;
  controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");
  name_ = controller_nh.getNamespace();
  controller_nh_ = controller_nh;
  const std::vector<std::string>& joint_names = robot_hw->get<hardware_interface::JointStateInterface>()->getNames();
  joint_names_ = joint_names;

  num_hw_joints_ = static_cast<int>(joint_names_.size());
  change_mode_server_ =
      controller_nh_.advertiseService("change_hybrid_mode", &ArmHybridController::changeHybridMode, this);
  // Init controller_state_publisher
  controller_state_interface_.init(controller_nh_, joint_names_);
  joints_interface_.init(robot_hw, controller_nh_);
  ros::NodeHandle nh_dynamics(controller_nh_, "dynamics");
  dynamics_interface_.init(nh_dynamics);

  // Action Service
  action_server_.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(
      controller_nh_, "follow_joint_trajectory", std::bind(&ArmHybridController::goalCB, this, std::placeholders::_1),
      std::bind(&ArmHybridController::cancelCB, this, std::placeholders::_1), false));
  action_server_->start();

  last_time_ = ros::Time::now();
  return true;
}

bool ArmHybridController::changeHybridMode(controller_msgs::ChangeHybridModeRequest& req,
                                           controller_msgs::ChangeHybridModeResponse& res)
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
  res.reply = "Now the mode is:" + mode_string;
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
void ArmHybridController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  joints_interface_.moveJoint(time, period);
}
void ArmHybridController::update(const ros::Time& time, const ros::Duration& period)
{
  controller_state_interface_.updateTimeData(time, period);
  dynamics_interface_.pubDynamics();
  controller_state_interface_.update(time, joints_interface_.jnt_states_);
  if (!is_enter_cb_)
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
        trajectory_tracking(time, period);
        break;
      case HOLDING_POSITION:
        holding_position(time);
        break;
    }
    last_time_ = time;
    if (dynamics_interface_.send_tau_)
      moveJoint(time, period);
  }
}
void ArmHybridController::gravity_compensation()
{
  dynamics_interface_.computerInverseDynamics(joints_interface_.jnt_states_);
  for (int i = 0; i < joints_interface_.num_hw_joints_; ++i)
  {
    joints_interface_.joints_[i].exe_effort_ = dynamics_interface_.tau_without_a_(i);
  }
}
void ArmHybridController::holding_position(const ros::Time& now)
{
  for (int i = 0; i < joints_interface_.num_hw_joints_; ++i)
  {
    double position_pid_value = joints_interface_.joints_[i].position_pid_->computeCommand(
        controller_state_interface_.state_error_.position[i], now - last_time_);
    double cmd = position_pid_value;
    joints_interface_.joints_[i].exe_effort_ = cmd;
  }
}
void ArmHybridController::trajectory_tracking(const ros::Time& time, const ros::Duration& period)
{
  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  if (curr_traj_ptr == nullptr)
  {
    ROS_INFO_STREAM("NO msg");
    mode_ = HOLDING_POSITION;
    controller_state_interface_.initDesiredState(joints_interface_.jnt_states_);
  }
  //    // Curr_traj is use for computer tolerance.
  else
  {
    Trajectory& curr_traj = *curr_traj_ptr;

    controller_state_interface_.old_time_data_ = *(controller_state_interface_.time_data_.readFromRT());
    // Update time data
    TimeData time_data;
    time_data.time = time;                                                          // Cache current time
    time_data.period = period;                                                      // Cache current control period
    time_data.uptime = controller_state_interface_.old_time_data_.uptime + period;  // Update controller uptime
    controller_state_interface_.time_data_.writeFromNonRT(time_data);

    controller_state_interface_.updateDesiredStates<Trajectory>(trajectory_points_time_, curr_traj_ptr.get());
    for (int i = 0; i < joints_interface_.num_hw_joints_; ++i)
    {
      double position_pid_value = joints_interface_.joints_[i].position_pid_->computeCommand(
          controller_state_interface_.state_error_.position[i], time - last_time_);
      double cmd = position_pid_value;
      joints_interface_.joints_[i].exe_effort_ = cmd;
    }
    ArmHybridController::updateFuncExtensionPoint(curr_traj, time_data);
    trajectory_points_time_ += period;
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
  is_enter_cb_ = true;
  ROS_INFO_STREAM("goalCB");
  mode_ = TRAJECTORY_TRACKING;
  //        Use for test trajectory without segment
  //    point_current_ = 0;
  points_ = gh.getGoal()->trajectory.points;
  new_gl_time_ = ros::Time::now();
  //        Use for test trajectory without segment
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  std::string error_string;
  const bool update_ok =
      updateTrajectoryCommand(internal::share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_goal, &error_string);
  rt_goal->preallocated_feedback_->joint_names = joints_interface_.joint_names_;
  if (update_ok)
  {
    // Accept new goal
    preemptActiveGoal();
    gh.setAccepted();
    rt_active_goal_ = rt_goal;
    // Setup goal status checking timer
    goal_handle_timer_ =
        controller_nh_.createTimer(action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
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
  is_enter_cb_ = false;
}

bool ArmHybridController::updateTrajectoryCommand(const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh,
                                                  std::string* error_string)
{
  // Time data
  TimeData* time_data = controller_state_interface_.time_data_.readFromRT();
  // Time of the next update
  const ros::Time next_update_time = time_data->time + time_data->period;
  // Uptime of the next update
  //    ros::Time next_update_uptime = time_data->uptime + time_data->period;

  // Hold current position if trajectory is empty
  if (msg->points.empty())
  {
    mode_ = HOLDING_POSITION;
    controller_state_interface_.initDesiredState(joints_interface_.jnt_states_);
    ROS_DEBUG_NAMED(name_, "Empty trajectory command, stopping.");
    return true;
  }

  // Trajectory initialization options
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);

  // Update currently executing trajectory
  std::string err_string;
  try
  {
    TrajectoryPtr traj_ptr(new Trajectory);
    *traj_ptr = joint_trajectory_controller::initJointTrajectory<Trajectory>(*msg, next_update_time);
    trajectory_points_time_ = msg.get()->header.stamp.now();
    if (!traj_ptr->empty())
    {
      curr_trajectory_box_.set(traj_ptr);
    }
    else
    {
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM_NAMED(name_, ex.what());
    return false;
  }
  return true;
}
}  // namespace arm_hybrid_controller