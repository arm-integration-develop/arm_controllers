//
// Created by lsy on 23-10-4.
//

#pragma once

#include "kinematics.h"
#include <string>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <effort_controllers/joint_position_controller.h>
#include <arm_common/tools/tf_rt_broadcaster.h>

namespace delta_controller
{
class Joint
{
public:
  std::string name_;
  double angle;
  effort_controllers::JointPositionController* position_ctrl_;
};
class DeltaController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                              hardware_interface::JointStateInterface>
{
public:
  DeltaController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period);
  void publishVel(const geometry_msgs::TransformStamped transform,
                  const geometry_msgs::TransformStamped start_transform);
  void commandCB(const geometry_msgs::PointStampedConstPtr& msg)
  {
    cmd_rt_buffer_.writeFromNonRT(*msg);
  }
  void velCmdCB(const geometry_msgs::TwistConstPtr& msg)
  {
    vel_cmd_ = *msg;
  }
  double timeout_{};
  bool use_gazebo_;
  std::vector<double> jnt_angle_{ 0., 0., 0. };
  std::vector<std::vector<double>> passive_jnt_angle_{};
  std::vector<std::string> joint_names_ = { "active_joint1", "active_joint2", "active_joint3" };
  std::vector<std::string> passive_joint_names_ = {
    "parallel_up_a_joint1", "parallel_up_b_joint1", "parallel_down_b_joint1", "parallel_down_a_joint1",
    "parallel_up_a_joint2", "parallel_up_b_joint2", "parallel_down_b_joint2", "parallel_down_a_joint2",
    "parallel_up_a_joint3", "parallel_up_b_joint3", "parallel_down_b_joint3", "parallel_down_a_joint3"
  };
  std::vector<Joint> joints_{}, passive_joints_{};
  std::vector<hardware_interface::JointStateHandle> jnt_states_;
  delta_controller::DeltaKinematics delta_kinematics_;

  tf::TfRtBroadcaster tf_broadcaster_;

  //    geometry_msgs::TwistStamped vel_msgs_{};
  geometry_msgs::Twist vel_msgs_{}, vel_cmd_{};
  ros::Publisher vel_pub_{};
  std::string RC_topic_;
  double coefficient_;
  double RC_start_value_;
  ros::NodeHandle node_;
  ros::Subscriber tf_sub_, cmd_subscriber_, vel_sub_;
  realtime_tools::RealtimeBuffer<geometry_msgs::PointStamped> cmd_rt_buffer_;
};

}  // namespace delta_controller

PLUGINLIB_EXPORT_CLASS(delta_controller::DeltaController, controller_interface::ControllerBase)