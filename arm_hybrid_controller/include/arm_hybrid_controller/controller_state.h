//
// Created by lsy on 23-11-1.
//
// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>
// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <trajectory_interface/quintic_spline_segment.h>
#pragma once
namespace arm_hybrid_controller
{
struct TimeData
{
    TimeData() : time(0.0), period(0.0), uptime(0.0) {}

    ros::Time     time;   ///< Time of last update cycle
    ros::Duration period; ///< Period of last update cycle
    ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
};
class ControllerStateInterface
{
public:
    void init(ros::NodeHandle& controller_nh,std::vector<std::string> joint_names)
    {
        double state_publish_rate = 50.0;
        controller_nh.getParam("state_publish_rate", state_publish_rate);
        state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);
        state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>(controller_nh, "state", 1));

        // Preeallocate resources
        int num_hw_joints = static_cast<int>(joint_names.size());
        current_state_       = trajectory_interface::QuinticSplineSegment<double>::State(num_hw_joints);
        old_desired_state_   = trajectory_interface::QuinticSplineSegment<double>::State(num_hw_joints);
        desired_state_       = trajectory_interface::QuinticSplineSegment<double>::State(num_hw_joints);
        state_error_         = trajectory_interface::QuinticSplineSegment<double>::State(num_hw_joints);
        desired_joint_state_ = trajectory_interface::QuinticSplineSegment<double>::State(1);
        state_joint_error_   = trajectory_interface::QuinticSplineSegment<double>::State(1);

        state_publisher_->lock();
        state_publisher_->msg_.joint_names = joint_names;
        state_publisher_->msg_.desired.positions.resize(num_hw_joints);
        state_publisher_->msg_.desired.velocities.resize(num_hw_joints);
        state_publisher_->msg_.desired.accelerations.resize(num_hw_joints);
        state_publisher_->msg_.actual.positions.resize(num_hw_joints);
        state_publisher_->msg_.actual.velocities.resize(num_hw_joints);
        state_publisher_->msg_.error.positions.resize(num_hw_joints);
        state_publisher_->msg_.error.velocities.resize(num_hw_joints);
        state_publisher_->unlock();
    }
    void publishState(const ros::Time& time)
    {
        // Check if it's time to publish
        if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time)
        {
            if (state_publisher_ && state_publisher_->trylock())
            {
                last_state_publish_time_ += state_publisher_period_;

                state_publisher_->msg_.header.stamp          = time_data_.readFromRT()->time;
                state_publisher_->msg_.desired.positions     = desired_state_.position;
                state_publisher_->msg_.desired.velocities    = desired_state_.velocity;
                state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
                state_publisher_->msg_.desired.time_from_start = ros::Duration(desired_state_.time_from_start);
                state_publisher_->msg_.actual.positions      = current_state_.position;
                state_publisher_->msg_.actual.velocities     = current_state_.velocity;
                state_publisher_->msg_.actual.time_from_start = ros::Duration(current_state_.time_from_start);
                state_publisher_->msg_.error.positions       = state_error_.position;
                state_publisher_->msg_.error.velocities      = state_error_.velocity;
                state_publisher_->msg_.error.time_from_start = ros::Duration(state_error_.time_from_start);

                state_publisher_->unlockAndPublish();
            }
        }
    }
    trajectory_interface::QuinticSplineSegment<double>::State current_state_;
    trajectory_interface::QuinticSplineSegment<double>::State desired_state_;
    trajectory_interface::QuinticSplineSegment<double>::State old_desired_state_;
    trajectory_interface::QuinticSplineSegment<double>::State state_error_;
    trajectory_interface::QuinticSplineSegment<double>::State desired_joint_state_;
    trajectory_interface::QuinticSplineSegment<double>::State state_joint_error_;

    ros::Duration state_publisher_period_;
    realtime_tools::RealtimeBuffer<TimeData> time_data_;
    ros::Time     last_state_publish_time_;
    std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>> state_publisher_;
};
}

