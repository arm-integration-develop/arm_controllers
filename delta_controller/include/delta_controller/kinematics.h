//
// Created by lsy on 23-10-4.
//

#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
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
  geometry_msgs::TransformStamped solveForwardKinematics(
    double theta1, double theta2, double theta3)
  {
    geometry_msgs::TransformStamped EE_tf;
    EE_tf.header.frame_id = "base_link";
    EE_tf.header.stamp = ros::Time::now();
    EE_tf.child_frame_id = "move_ee";
    //      the algorithm of forward kinematics
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double t = (parameter_.f - parameter_.e) * tan(M_PI / 6) / 2.0;

    double y1 = -(t + parameter_.r_f * cos(theta1));
    double z1 = -parameter_.r_f * sin(theta1);

    double y2 = (t + parameter_.r_f * cos(theta2)) * sin(M_PI / 6);
    double x2 = y2 * tan(M_PI / 3);
    double z2 = -parameter_.r_f * sin(theta2);

    double y3 = (t + parameter_.r_f * cos(theta3)) * sin(M_PI / 6);
    double x3 = -y3 * tan(M_PI / 3);
    double z3 = -parameter_.r_f * sin(theta3);

    double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    double w1 = y1 * y1 + z1 * z1;
    double w2 = x2 * x2 + y2 * y2 + z2 * z2;
    double w3 = x3 * x3 + y3 * y3 + z3 * z3;

    // x = (a1*z + b1)/dnm
    double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

    // y = (a2*z + b2)/dnm;
    double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

    // a*z^2 + b*z + c = 0
    double aV = a1 * a1 + a2 * a2 + dnm * dnm;
    double bV = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
    double cV = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 +
                dnm * dnm * (z1 * z1 - parameter_.r_e * parameter_.r_e);

    // discriminant
    double dV = bV * bV - 4.0 * aV * cV;
    //    if (dV < 0.0) {
    //      return non_existing_povar_error;  // non-existing povar. return error,x,y,z
    //    }

    z = -0.5 * (bV + sqrt(dV)) / aV;
    x = (a1 * z + b1) / dnm;
    y = (a2 * z + b2) / dnm;

    EE_tf.transform.translation.x = x;
    EE_tf.transform.translation.y = y;
    EE_tf.transform.translation.z = z;
    EE_tf.transform.rotation.x = 0;
    EE_tf.transform.rotation.y = 0;
    EE_tf.transform.rotation.z = 0;
    EE_tf.transform.rotation.w = 1;
    return EE_tf;
  }
  std::vector<double> solveInverseKinematics(double x, double y, double z)
  {
    //        ROS_INFO_STREAM("solve ik");
    std::vector<double> jnt_angle{0., 0., 0.};
    std::vector<geometry_msgs::Point> j1_position = getJ1Position(x, y, z);
    for (int i = 0; i < 3; ++i) {
      jnt_angle[i] = atan(-j1_position[i].z / (-0.5 * 0.57735 * parameter_.f - j1_position[i].y)) +
                     ((j1_position[i].y > -0.5 * 0.57735 * parameter_.f) ? M_PI : 0.0);
    }
    return jnt_angle;
  }
  std::vector<std::vector<double>> getPassiveAngle(double x, double y, double z)
  {
    double tan_30_deg = 0.5773502692;
    double y_F = -parameter_.f / 2 * tan_30_deg;
    double y_delta_E = parameter_.e / 2 * tan_30_deg;
    std::vector<double> alpha_rad{0., 120 * M_PI / 180, 240 * M_PI / 180};
    std::vector<std::vector<double>> passive_angle{
      {0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 0.}};
    std::vector<geometry_msgs::Point> j1_position = getJ1Position(x, y, z);
    std::vector<double> active_jnt = solveInverseKinematics(x, y, z);
    for (int i = 0; i < (int)j1_position.size(); ++i) {
      std::vector<double> transform_xyz{x, y, z};
      transform_xyz[0] = x * cos(alpha_rad[i]) + y * sin(alpha_rad[i]);
      transform_xyz[1] = -x * sin(alpha_rad[i]) + y * cos(alpha_rad[i]);
      transform_xyz[2] = z;
      //          C = acos((a2+b2-c2)/2ab)
      std::vector<double> passive_joint{0., 0., 0., 0.};
      double FJ = parameter_.r_f;
      //            double JE = parameter_.r_e;
      //            double FE = sqrt(pow2(y_F-y-y_delta_E)+pow2(x)+ pow2(z));
      double FE_prime = sqrt(pow2(transform_xyz[2]) + pow2(y_F - (transform_xyz[1] - y_delta_E)));
      double JE_prime = sqrt(
        pow2(j1_position[i].z - transform_xyz[2]) +
        pow2(j1_position[i].y - (transform_xyz[1] - y_delta_E)));
      //            double other_JE_prime = pow2(JE) - pow2(transform_xyz[0]);
      //            double JO = sqrt(pow2(j1_position[i].x-transform_xyz[0])+pow2(j1_position[i].y-transform_xyz[1])+pow2(j1_position[i].z-transform_xyz[2]));
      //            double OE = y_delta_E;
      passive_joint[0] = M_PI / 3 - cosineTheorem(FJ, JE_prime, FE_prime);
      passive_joint[1] = -atan(transform_xyz[0] / JE_prime);
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
    std::vector<double> alpha_rad{0., 120 * M_PI / 180, 240 * M_PI / 180};
    std::vector<geometry_msgs::Point> j1_position;
    j1_position.clear();
    for (int i = 0; i < 3; ++i) {
      double x0 = x * cos(alpha_rad[i]) + y * sin(alpha_rad[i]);
      double y0 = -x * sin(alpha_rad[i]) + y * cos(alpha_rad[i]);
      double z0 = z;

      double y1 = -0.5 * 0.57735 * parameter_.f;  // f/2 * tan(30 deg)
      y0 -= 0.5 * 0.57735 * parameter_.e;         // shift center to edge

      // z = a + b*y
      double aV = (x0 * x0 + y0 * y0 + z0 * z0 + parameter_.r_f * parameter_.r_f -
                   parameter_.r_e * parameter_.r_e - y1 * y1) /
                  (2.0 * z0);
      double bV = (y1 - y0) / z0;

      // discriminant
      double dV = -(aV + bV * y1) * (aV + bV * y1) +
                  parameter_.r_f * (bV * bV * parameter_.r_f + parameter_.r_f);

      double yj, zj;
      if (dV > 0) {
        yj = (y1 - aV * bV - sqrt(dV)) / (bV * bV + 1);  // choosing outer povar
        zj = aV + bV * yj;
      }

      geometry_msgs::Point solution_point;
      solution_point.x = 0.;
      solution_point.y = yj;
      solution_point.z = zj;
      j1_position.push_back(solution_point);
    }
    return j1_position;
  }
  double pow2(double input) { return pow(input, 2); }
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