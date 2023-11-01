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

namespace arm_hybrid_controller
{
class Joint
{
public:
    std::string name_;
    effort_controllers::JointEffortController* effort_ctrl_;
};

class ArmHybridController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
        hardware_interface::JointStateInterface>
{
public:
    ArmHybridController() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
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
    void holding_position();

    enum
    {
        GRAVITY_COMPENSATION,
        TRAJECTORY_TEACHING,
        TRAJECTORY_TRACKING,
        HOLDING_POSITION
    };
    int mode_ = GRAVITY_COMPENSATION;
    dynamics_interface::DynamicsInterface dynamics_interface_;
    ros::NodeHandle node_;
    int num_hw_joints_;
    std::vector<std::string> joint_names_{};
    std::vector<hardware_interface::JointStateHandle> jnt_states_;
    std::vector<Joint> joints_{};
};
}// namespace arm_hybrid_controller

PLUGINLIB_EXPORT_CLASS(arm_hybrid_controller::ArmHybridController, controller_interface::ControllerBase)