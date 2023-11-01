//
// Created by lsy on 23-10-25.
//

#pragma once

#include <dynamics_interface/dynamics_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <effort_controllers/joint_effort_controller.h>

#include <string>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <tools/lp_filter.h>
#include <arm_hybrid_controller/controller_state.h>
#include <arm_hybrid_controller/joints_interface.h>

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
    void gravity_compensation();
    void trajectory_teaching();
    void trajectory_tracking();
    void holding_position(const ros::Time& now);
    enum
    {
        GRAVITY_COMPENSATION,
        TRAJECTORY_TEACHING,
        TRAJECTORY_TRACKING,
        HOLDING_POSITION
    };
    int mode_ = GRAVITY_COMPENSATION;
    ros::Time last_time_;
    dynamics_interface::DynamicsInterface dynamics_interface_;
    JointsInterface joints_interface_;
    ControllerStateInterface controller_state_interface_{};
};
}// namespace arm_hybrid_controller

PLUGINLIB_EXPORT_CLASS(arm_hybrid_controller::ArmHybridController, controller_interface::ControllerBase)