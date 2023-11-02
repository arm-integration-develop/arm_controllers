//
// Created by lsy on 23-10-25.
//

#pragma once

#include <functional>

// dynamics
#include <dynamics_interface/dynamics_interface.h>

// controller_interface
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <effort_controllers/joint_effort_controller.h>

// messages
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_msgs/ChangeHybridMode.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// actionlib
#include <actionlib/server/action_server.h>

// some custom interface
#include <tools/lp_filter.h>
#include <arm_hybrid_controller/controller_state_interface.h>
#include <arm_hybrid_controller/joints_interface.h>

// realtime_tools
#include <realtime_tools/realtime_server_goal_handle.h>
namespace arm_hybrid_controller
{
class ArmHybridController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
        hardware_interface::JointStateInterface>
{
public:
    ArmHybridController() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

private:
    void moveJoint(const ros::Time& time, const ros::Duration& period);
    void commandCB(const geometry_msgs::PointStampedConstPtr & msg)
    {
//        cmd_rt_buffer_.writeFromNonRT(*msg);
    }
    bool changeHybridMode(controller_msgs::ChangeHybridModeRequest &req,controller_msgs::ChangeHybridModeResponse &res);
    void gravity_compensation();
    void trajectory_teaching();
    void trajectory_tracking();
    void holding_position(const ros::Time& now);

    // Action function
    void goalCB(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh,ros::NodeHandle &controller_nh)
    {
        gh.getGoal();

    }
    void cancelCB(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
    {
        ROS_INFO_STREAM(gh.getGoalID());
    }
    enum
    {
        GRAVITY_COMPENSATION,
        TRAJECTORY_TEACHING,
        TRAJECTORY_TRACKING,
        HOLDING_POSITION
    };

    int mode_;
    ros::Time last_time_;
    ros::Timer goal_handle_timer_;

    ros::ServiceServer change_mode_server_;
    ros::Duration action_monitor_period_;
    std::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> action_server_;

    dynamics_interface::DynamicsInterface dynamics_interface_;
    JointsInterface joints_interface_;
    ControllerStateInterface controller_state_interface_{};
};
}// namespace arm_hybrid_controller

PLUGINLIB_EXPORT_CLASS(arm_hybrid_controller::ArmHybridController, controller_interface::ControllerBase)