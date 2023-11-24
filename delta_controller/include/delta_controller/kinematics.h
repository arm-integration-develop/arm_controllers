//
// Created by lsy on 23-10-4.
//

#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <ros/ros.h>

namespace delta_controller
{
class DeltaKinematicsParameter
{
public:
  DeltaKinematicsParameter() = default;
  double f, r_f, r_e, e;
};
class DeltaKinematics
{
public:
  DeltaKinematics() = default;
  void init(ros::NodeHandle nh_param)
  {
    nh_param.getParam("f", parameter_.f);
    nh_param.getParam("r_f", parameter_.r_f);
    nh_param.getParam("r_e", parameter_.r_e);
    nh_param.getParam("e", parameter_.e);
  }
  geometry_msgs::TransformStamped solveForwardKinematics(double theta1, double theta2, double theta3)
  {
    geometry_msgs::TransformStamped EE_tf;
    EE_tf.header.frame_id = "base_link";
    EE_tf.header.stamp = ros::Time::now();
    EE_tf.child_frame_id = "move_ee";
    //      the algorithm of forward kinematics
    double sin_30_deg = 0.5;
    double cos_30_deg = sqrt(3) / 2;
    double y_F = parameter_.f / (2 * sqrt(3));
    double y_delta_E = parameter_.e / (2 * sqrt(3));
    geometry_msgs::Point F1, F2, F3, J1, J2, J3, J1_prime, J2_prime, J3_prime, solution;
    F1.x = 0;
    F1.y = -y_F;
    F1.z = 0;
    F2.x = y_F * cos_30_deg;
    F2.y = y_F * sin_30_deg;
    F2.z = 0;
    F3.x = -y_F * cos_30_deg;
    F3.y = y_F * sin_30_deg;
    F3.z = 0;
    J1 = F1;
    J1.y -= parameter_.r_f * cos(theta1);
    J1.z -= parameter_.r_f * sin(theta1);
    J2 = F2;
    J2.x += parameter_.r_f * cos(theta2) * cos_30_deg;
    J2.y += parameter_.r_f * cos(theta2) * sin_30_deg;
    J2.z -= parameter_.r_f * sin(theta2);
    J3 = F3;
    J3.x -= parameter_.r_f * cos(theta3) * cos_30_deg;
    J3.y += parameter_.r_f * cos(theta3) * sin_30_deg;
    J3.z -= parameter_.r_f * sin(theta3);
    J1_prime = J1;
    J1_prime.y += y_delta_E;
    J2_prime = J2;
    J2_prime.x -= y_delta_E * cos_30_deg;
    J2_prime.y -= y_delta_E * sin_30_deg;
    J3_prime = J3;
    J3_prime.x += y_delta_E * cos_30_deg;
    J3_prime.y -= y_delta_E * sin_30_deg;
    double W1 = pow2(J1_prime.x) + pow2(J1_prime.y) + pow2(J1_prime.z);
    double W2 = pow2(J2_prime.x) + pow2(J2_prime.y) + pow2(J2_prime.z);
    double W3 = pow2(J3_prime.x) + pow2(J3_prime.y) + pow2(J3_prime.z);
    double d = (J2_prime.y - J1_prime.y) * J3_prime.x - (J3_prime.y - J1_prime.y) * J2_prime.x;
    double a1 = (1 / d) * ((J2_prime.z - J1_prime.z) * (J3_prime.y - J1_prime.y) -
                           (J3_prime.z - J1_prime.z) * (J2_prime.y - J1_prime.y));
    double a2 = (-1 / d) * ((J2_prime.z - J1_prime.z) * J3_prime.x - (J3_prime.z - J1_prime.z) * J2_prime.x);
    double b1 = (1 / (2 * d)) * ((W2 - W1) * (J3_prime.y - J1_prime.y) - (W3 - W1) * (J2_prime.y - J1_prime.y));
    double b2 = (1 / (2 * d)) * ((W2 - W1) * J3_prime.x - (W3 - W1) * J2_prime.x);
    //      solution the z value
    double a = pow2(a1) + pow2(a2) + 1;
    double b = 2 * (a1 * b1 + a2 * (b2 - J1_prime.y) - J1_prime.z);
    double c = (pow2(b1) + pow2(b2 - J1_prime.y) + pow2(J1_prime.z) - pow2(parameter_.r_e));
    solution.z = quadraticFormulaSmall(a, b, c);
    solution.x = a1 * solution.z + b1;
    solution.y = a2 * solution.z + b2;
    EE_tf.transform.translation.x = solution.x;
    EE_tf.transform.translation.y = solution.y;
    EE_tf.transform.translation.z = solution.z;
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
    std::vector<double> jnt_angle{ 0., 0., 0. };
    std::vector<geometry_msgs::Point> j1_position = getJ1Position(x, y, z);
    for (int i = 0; i < 3; ++i)
    {
      jnt_angle[i] = -atan(j1_position[i].z / (parameter_.f / 2 * tan_30_deg - j1_position[i].y));
      //            ROS_INFO_STREAM(jnt_angle[i]);
    }
    return jnt_angle;
  }

  std::vector<double> solveVelocity(double vel_x, double vel_y, double vel_z,
                                    std::vector<std::vector<double>> passive_angle, std::vector<double> theta_1_i)
  {
    std::vector<double> joints_vel;
    std::vector<double> end_vel{ vel_x, vel_y, vel_z };
    std::vector<double> alpha_rad{ 0., 120 * M_PI / 180, 240 * M_PI / 180 };
    std::vector<double> Jx, Jy, Jz, phi;
    for (int i = 0; i < 3; i++)
    {
      phi.push_back(alpha_rad[i] + M_PI / 6);
      Jx.push_back(sin(passive_angle[i][1]) * cos(abs(theta_1_i[0]) + passive_angle[i][0]) * cos(phi[i]) +
                   cos(passive_angle[i][1]) * sin(phi[i]));
      Jy.push_back(-sin(passive_angle[i][1]) * cos(abs(theta_1_i[0]) + passive_angle[i][0]) * sin(phi[i]) +
                   cos(passive_angle[i][1]) * cos(phi[i]));
      Jz.push_back(sin(passive_angle[i][1]) * sin(abs(theta_1_i[0]) + passive_angle[i][0]));
    }

    Eigen::Matrix<double, 3, 3> J_theta, J_p;
    Eigen::Matrix<double, 3, 1> vel, theta_dot;

    // clang-format off
    J_theta <<  sin(passive_angle[0][0])*sin(passive_angle[0][1]),  0,  0,
                0,  sin(passive_angle[1][0])*sin(passive_angle[1][1]),  0,
                0,  0,  sin(passive_angle[2][0])*sin(passive_angle[2][1]);

    J_p <<  Jx[0],  Jy[0],  Jz[0],
            Jx[1],  Jy[1],  Jz[1],
            Jx[2],  Jy[2],  Jz[2];

    vel <<  vel_x,
            vel_y,
            vel_z;
    // clang-format on

    theta_dot = J_theta.inverse() * J_p * vel;
    for (int i = 0; i < 3; i++)
    {
      joints_vel.push_back(theta_dot[i]);
      ROS_INFO_STREAM(i << " " << theta_dot[i]);
    }
    return joints_vel;
  }

  std::vector<std::vector<double>> getPassiveAngle(double x, double y, double z)
  {
    double tan_30_deg = 0.5773502692;
    double y_F = -parameter_.f / 2 * tan_30_deg;
    double y_delta_E = parameter_.e / 2 * tan_30_deg;
    std::vector<double> alpha_rad{ 0., 120 * M_PI / 180, 240 * M_PI / 180 };
    std::vector<std::vector<double>> passive_angle{ { 0., 0., 0., 0. }, { 0., 0., 0., 0. }, { 0., 0., 0., 0. } };
    std::vector<geometry_msgs::Point> j1_position = getJ1Position(x, y, z);
    std::vector<double> active_jnt = solveInverseKinematics(x, y, z);
    for (int i = 0; i < (int)j1_position.size(); ++i)
    {
      std::vector<double> transform_xyz{ x, y, z };
      transform_xyz[0] = x * cos(alpha_rad[i]) + y * sin(alpha_rad[i]);
      transform_xyz[1] = -x * sin(alpha_rad[i]) + y * cos(alpha_rad[i]);
      transform_xyz[2] = z;
      //          C = acos((a2+b2-c2)/2ab)
      std::vector<double> passive_joint{ 0., 0., 0., 0. };
      double FJ = parameter_.r_f;
      //            double JE = parameter_.r_e;
      //            double FE = sqrt(pow2(y_F-y-y_delta_E)+pow2(x)+ pow2(z));
      double FE_prime = sqrt(pow2(transform_xyz[2]) + pow2(y_F - (transform_xyz[1] - y_delta_E)));
      double JE_prime =
          sqrt(pow2(j1_position[i].z - transform_xyz[2]) + pow2(j1_position[i].y - (transform_xyz[1] - y_delta_E)));
      //            double other_JE_prime = pow2(JE) - pow2(transform_xyz[0]);
      //            double JO =
      //            sqrt(pow2(j1_position[i].x-transform_xyz[0])+pow2(j1_position[i].y-transform_xyz[1])+pow2(j1_position[i].z-transform_xyz[2]));
      //            double OE = y_delta_E;
      passive_joint[0] = M_PI - cosineTheorem(FJ, JE_prime, FE_prime);
      passive_joint[1] = M_PI / 2 - atan(transform_xyz[0] / JE_prime);
      passive_joint[2] = -passive_joint[1];
      passive_joint[3] = -passive_joint[0] - active_jnt[i];
      passive_angle[i] = passive_joint;
    }
    return passive_angle;
  }
  double cosineTheorem(double a, double b, double c)
  {
    double cosC = (pow2(a) + pow2(b) - pow2(c)) / (2 * a * b);
    double angleC = acos(cosC);
    return angleC;
  }
  std::vector<geometry_msgs::Point> getJ1Position(double x, double y, double z)
  {
    double tan_30_deg = 0.5773502692;
    //        double y_delta_E = parameter_.e/2*tan_30_deg;
    std::vector<double> alpha_rad{ 0., 120 * M_PI / 180, 240 * M_PI / 180 };
    std::vector<geometry_msgs::Point> j1_position;
    j1_position.clear();
    for (int i = 0; i < 3; ++i)
    {
      std::vector<double> EE_position{ 3, 0 };
      EE_position[0] = x * cos(alpha_rad[i]) + y * sin(alpha_rad[i]);
      EE_position[1] = -x * sin(alpha_rad[i]) + y * cos(alpha_rad[i]);
      EE_position[2] = z;
      std::vector<double> F1_position{ 0., -parameter_.f / 2 * tan_30_deg, 0. };
      std::vector<double> E1_position{ EE_position[0], EE_position[1] - parameter_.e / 2 * tan_30_deg, EE_position[2] };
      std::vector<double> E1_prime_position{ 0., E1_position[1], E1_position[2] };
      double y_F = F1_position[1];
      double c1 = (pow2(E1_position[0]) + pow2(E1_position[1]) + pow2(E1_position[2]) + pow2(parameter_.r_f) -
                   pow2(parameter_.r_e) - pow2(y_F)) /
                  (2 * E1_position[2]);
      double c2 = (y_F - E1_position[1]) / E1_position[2];

      double c3 = -pow2(c1 + c2 * y_F) + pow2(parameter_.r_f) * (pow2(c2) + 1);
      double J_y = ((y_F - c1 * c2) - sqrt(c3)) / (1 + pow2(c2));
      double J_z = c1 + c2 * J_y;

      geometry_msgs::Point solution_point;
      solution_point.x = 0.;
      solution_point.y = J_y;
      solution_point.z = J_z;
      //            ROS_INFO_STREAM(J_z);
      j1_position.push_back(solution_point);
    }
    return j1_position;
  }
  double pow2(double input)
  {
    return pow(input, 2);
  }
  double quadraticFormulaSmall(double a, double b, double c)
  {
    double solution_small;
    solution_small = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
    return solution_small;
  }
  double quadraticFormulaBig(double a, double b, double c)
  {
    double solution_big;
    solution_big = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
    return solution_big;
  }

private:
  DeltaKinematicsParameter parameter_;
};
}  // namespace delta_controller