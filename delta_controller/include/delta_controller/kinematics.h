//
// Created by lsy on 23-10-4.
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

namespace delta_controller
{
class DeltaKinematicsParameter
{
public:
    DeltaKinematicsParameter() = default;
    double f, r_f, r_e, e;
};
class DeltaKinematics {
public:
    DeltaKinematics() = default;
    void init(ros::NodeHandle nh_param)
    {
        nh_param.getParam("f",parameter_.f);
        nh_param.getParam("r_f",parameter_.r_f);
        nh_param.getParam("r_e",parameter_.r_e);
        nh_param.getParam("e",parameter_.e);
    }
    geometry_msgs::TransformStamped solveForwardKinematics(double theta1, double theta2, double theta3)
    {
        geometry_msgs::TransformStamped EE_tf;
        EE_tf.header.frame_id = "static_base";
        EE_tf.header.stamp = ros::Time::now();
        EE_tf.child_frame_id = "move_ee";
        geometry_msgs::Point solution;
//      the algorithm of forward kinematics
        solution.x = 1;
        solution.y = 2;
        solution.z = 3;
//
        EE_tf.transform.translation.x = solution.x;
        EE_tf.transform.translation.y = solution.y;
        EE_tf.transform.translation.y = solution.z;
        EE_tf.transform.rotation.x = 0;
        EE_tf.transform.rotation.y = 0;
        EE_tf.transform.rotation.z = 0;
        EE_tf.transform.rotation.w = 1;
        return EE_tf;
    }
    std::vector<double> solveInverseKinematics(double x, double y, double z)
    {
//        ROS_INFO_STREAM("solve ik");
        double tan_30_deg = 0.5773502692;
        std::vector<double> jnt_angle{0.,0.,0.};
        std::vector<geometry_msgs::Point> j1_position = getJ1Position(x,y,z);
        for (int i = 0; i < 3; ++i)
        {
            jnt_angle[i] = -atan(j1_position[i].z/(parameter_.f/2*tan_30_deg-j1_position[i].y));
//            ROS_INFO_STREAM(jnt_angle[i]);
        }
        return  jnt_angle;
    }
    std::vector<std::vector<double>> getPassiveAngle(double x, double y, double z)
    {
        double tan_30_deg = 0.5773502692;
        double y_F = -parameter_.f/2*tan_30_deg;
        double y_delta_E = parameter_.e/2*tan_30_deg;
        std::vector<double> alpha_rad{0., 240*M_PI/180, 120*M_PI/180};
        std::vector<std::vector<double>> passive_angle{{0.,0.,0.,0.},{0.,0.,0.,0.},{0.,0.,0.,0.}};
        std::vector<geometry_msgs::Point> j1_position = getJ1Position(x,y,z);
        std::vector<double> active_jnt = solveInverseKinematics(x,y,z);
        for (int i = 0; i < (int)j1_position.size() ; ++i) {
            std::vector<double> transform_xyz{x,y,z};
            transform_xyz[0] = x*cos(alpha_rad[i]) + y*sin(alpha_rad[i]);
            transform_xyz[1] = -x*sin(alpha_rad[i]) + y*cos(alpha_rad[i]);
            transform_xyz[2] = z;
            //          C = acos((a2+b2-c2)/2ab)
            std::vector<double> passive_joint{0.,0.,0.,0.};
            double FJ = parameter_.r_f;
//            double JE = parameter_.r_e;
//            double FE = sqrt(pow2(y_F-y-y_delta_E)+pow2(x)+ pow2(z));
            double FE_prime = sqrt(pow2(transform_xyz[2])+ pow2(y_F-(transform_xyz[1]-y_delta_E)));
            double JE_prime = sqrt(pow2(j1_position[i].z-transform_xyz[2])+ pow2(j1_position[i].y-(transform_xyz[1]-y_delta_E)));
//            double other_JE_prime = pow2(JE) - pow2(transform_xyz[0]);
//            double JO = sqrt(pow2(j1_position[i].x-transform_xyz[0])+pow2(j1_position[i].y-transform_xyz[1])+pow2(j1_position[i].z-transform_xyz[2]));
//            double OE = y_delta_E;
            passive_joint[0] = M_PI/3-cosineTheorem(FJ,JE_prime,FE_prime);
            passive_joint[1] = -atan(transform_xyz[0]/JE_prime);
            passive_joint[2] = -passive_joint[1];
            passive_joint[3] = -passive_joint[0] - active_jnt[i];
            passive_angle[i] = passive_joint;
        }
        return passive_angle;
    }
    double cosineTheorem(double a,double b,double c)
    {
        double cosC = (pow2(a)+ pow2(b)- pow2(c))/(2*a*b);
        double angleC = acos(cosC);
        return angleC;
    }
    std::vector<geometry_msgs::Point> getJ1Position(double x, double y, double z)
    {
        double tan_30_deg = 0.5773502692;
//        double y_delta_E = parameter_.e/2*tan_30_deg;
        std::vector<double> alpha_rad{0., 240*M_PI/180, 120*M_PI/180};
        std::vector<geometry_msgs::Point> j1_position;
        j1_position.clear();
        for (int i = 0; i < 3; ++i) {
            std::vector<double> EE_position{3,0};
            EE_position[0] = x*cos(alpha_rad[i]) + y*sin(alpha_rad[i]);
            EE_position[1] = -x*sin(alpha_rad[i]) + y*cos(alpha_rad[i]);
            EE_position[2] = z;
            std::vector<double> F1_position{0.,  -parameter_.f/2*tan_30_deg, 0.};
            std::vector<double> E1_position{EE_position[0],EE_position[1]-parameter_.e/2*tan_30_deg,EE_position[2]};
            std::vector<double> E1_prime_position{0.,E1_position[1],E1_position[2]};
            double y_F = F1_position[1];
            double c1 = (pow2(E1_position[0]) + pow2(E1_position[1]) + pow2(E1_position[2]) + pow2(parameter_.r_f)
                    - pow2(parameter_.r_e)- pow2(y_F))/(2*E1_position[2]);
            double c2 = (E1_position[1] - y_F)/E1_position[2];

            double c3 = -pow2(c1+c2*y_F) + pow2(parameter_.r_f)*(pow2(c2)+1);
            double J_y = ((y_F-c1*c2)- sqrt(c3))/(1+ pow2(c2));
            double J_z = c1+c2*J_y;

//            double c1 = (pow2(y_F)- pow2(EE_position[1])-2*y_delta_E*EE_position[1]- pow2(y_delta_E)- pow2(parameter_.r_f) + pow2(parameter_.r_e)-pow2(EE_position[0]))/(2*EE_position[2]);
//            double c2 = (y_F+EE_position[1]-y_delta_E)/EE_position[2];
//            double c3 = 2*y_F*c1*c2-pow2(c1)+ pow2(parameter_.r_f)*(1+ pow2(c2))-pow2(y_F)* pow2(c2);
//            if (c3<0)
//            {
//                ROS_INFO_STREAM("NO EXISTING POINT");
//            }
//            double J_y = (-1*(y_F+c1*c2)- sqrt(c3))/(1+ pow2(c2));
//            double J_z = -(c1+c2*J_y);
            geometry_msgs::Point solution_point;
            solution_point.x =0.;
            solution_point.y = J_y;
            solution_point.z = J_z;
//            ROS_INFO_STREAM(J_z);
            j1_position.push_back(solution_point);
        }
        return j1_position;
    }
    double pow2(double input)
    {
        return pow(input,2);
    }


private:
    DeltaKinematicsParameter parameter_;

};
} // namespace delta_controller