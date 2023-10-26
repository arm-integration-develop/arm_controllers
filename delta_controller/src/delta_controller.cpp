//
// Created by lsy on 23-10-4.
//

#include "delta_controller/delta_controller.h"

namespace delta_controller
{
bool DeltaController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
{
    controller_nh.getParam("use_gazebo",use_gazebo_);
    realtime_tf_pub_.init(node_, "/tf", 100);
    tf_pub_ = controller_nh.advertise<geometry_msgs::TransformStamped>("/tf",1);
    hardware_interface::EffortJointInterface* effort_joint_interface;
    effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    ros::NodeHandle nh_active(controller_nh, "active");
    for (const auto& joint_name : joint_names_)
    {
        Joint j{ .name_ = joint_name,
                .position_ctrl_ = new effort_controllers::JointPositionController()};
        ros::NodeHandle nh_position(nh_active, joint_name);
        j.position_ctrl_->init(effort_joint_interface, nh_position);
        joints_.push_back(j);
    }
    if (use_gazebo_)
    {
        ros::NodeHandle nh_passive(controller_nh, "passive");
        for (const auto& joint_name : passive_joint_names_)
        {
            Joint j{ .name_ = joint_name,
                    .position_ctrl_ = new effort_controllers::JointPositionController()};
            ros::NodeHandle nh_position(nh_passive, joint_name);
            j.position_ctrl_->init(effort_joint_interface, nh_position);
            passive_joints_.push_back(j);
        }
    }
    ros::NodeHandle nh_param(controller_nh, "param");
    delta_kinematics_.init(nh_param);
    cmd_subscriber_ = controller_nh.subscribe<geometry_msgs::PointStamped>("command", 1, &DeltaController::commandCB, this);
    return true;
}
void DeltaController::update(const ros::Time &time, const ros::Duration &period)
{
    //get zhe act angle from /tf
    double theta1 =0, theta2= 0, theta3= 0;
    geometry_msgs::TransformStamped EE_tf = delta_kinematics_.solveForwardKinematics(theta1, theta2, theta3);
    publishTF(EE_tf);
    geometry_msgs::Point cmd = cmd_rt_buffer_.readFromRT()->point;
//    if (judgeWorkSpace(cmd))
//    {
//        for (auto it = joints_.begin() ; it != joints_.end() ; ++it, ++i)
//            it->angle = it->position_ctrl_->getPosition();
//    }
    //judge if timeout
//    if ((time - cmd_rt_buffer_.readFromRT()->header.stamp).toSec() > timeout_)
//    {
//    }
//    else
//    {
        jnt_angle_ = delta_kinematics_.solveInverseKinematics(cmd.x, cmd.y, cmd.z);
//        ROS_INFO_STREAM(cmd);
//        for (int j = 0; j < (int)jnt_angle_.size(); ++j) {
//            ROS_INFO_STREAM(jnt_angle_[j]);
//        }
        int i = 0;
        for (auto it = joints_.begin() ; it != joints_.end() ; ++it, ++i)
            it->angle = jnt_angle_[i];
        if (use_gazebo_)
        {
            passive_jnt_angle_ = delta_kinematics_.getPassiveAngle(cmd.x, cmd.y, cmd.z);
            int j = 0;
            for (auto it = passive_joints_.begin() ; it != passive_joints_.end() ; ++it, ++j)
            {
                it->angle = passive_jnt_angle_[j/4][j-(j/4)*4];
//                ROS_INFO_STREAM(it->angle);
            }
        }
//    }
    // Add trajectory planning
    moveJoint(time,period);
}
void DeltaController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
    for (const auto& joint : joints_)
    {
//        ROS_INFO_STREAM(joint.angle);
        joint.position_ctrl_->setCommand(joint.angle);
        joint.position_ctrl_->update(time, period);
    }
    if (use_gazebo_)
    {
        for (const auto& joint : passive_joints_)
        {
//            ROS_INFO_STREAM(joint.angle);
            joint.position_ctrl_->setCommand(joint.angle);
            joint.position_ctrl_->update(time, period);
        }
    }
}

} // namespace delta_controller

