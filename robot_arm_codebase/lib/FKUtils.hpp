#ifdef ARDUINO
#include <ArduinoEigenDense.h>
#else
#include <Eigen/Dense>
#endif

using namespace Eigen;
using Eigen::MatrixXd;

const double d1 = 167.719;
const double d2 = 19.898;
const double d3 = -0.39;
const double d4 = 231.03; // 177.485 + 54.70;
const double d5 = 2.785;
const double d6 = 140.54; // 95.069 + 58.60; // end effector ball centre 58.60

const double FK_a1 = 0;       // 23.216 centre offset
const double FK_a2 = 0;       // length of lower arm
const double FK_a3 = 250.201; // length of upper arm
const double FK_a4 = 0;
const double FK_a5 = 0;
const double FK_a6 = 0;

const double alpha1 = 0;
const double alpha2 = rad(90);
const double alpha3 = 0;
const double alpha4 = rad(90);
const double alpha5 = -rad(90);
const double alpha6 = rad(90);

const double theta1_offset = 0;
const double theta2_offset = 0;
const double theta3_offset = 0;
const double theta4_offset = 0;
const double theta5_offset = 0;
const double theta6_offset = 0;

Matrix4d createDHMatrix(float theta, float d, float a, float alpha)
{
  /*
  θ​	------  Joint angle — rotation around the Z-axis (for revolute joints)
  d​	------  Offset — translation along Z-axis
  a(r)  ---  Link length — distance along the X-axis from this joint to next
  alpha​ ---  Twist angle — rotation about X-axis (defines the twist between axes)

  ai = distance along xi from the intersection of the xi and zi − 1 axes to .
  di = distance along zi − 1 from to the intersection of the xi and zi − 1 axes. If joint i is prismatic, di is variable.
  αi = the angle from zi − 1 to zi measured about xi.
  θi = the angle from xi − 1 to xi measured about zi − 1. If joint i is revolute, θi is variable

  Chapter 3.2, fig 3.10:
  Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). Robot modeling and control (Second edition.). John Wiley & Sons, Inc.
  https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
  */

  // Generic DH matrix and Modified version:

  // return Matrix4d{
  //     {cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)},
  //     {sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)},
  //     {0, sin(alpha), cos(alpha), d},
  //     {0, 0, 0, 1}};

  return Matrix4d{
      {cos(theta), -sin(theta), 0, a},
      {sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -d * sin(alpha)},
      {sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), d * cos(alpha)},
      {0, 0, 0, 1}};
}

pair<Coor, vector<Matrix4d>> FK(const JointAngle &angles)
{
  Matrix4d T1 = createDHMatrix(angles.theta1 + theta1_offset, d1, FK_a1, alpha1);
  Matrix4d T2 = createDHMatrix(angles.theta2 + theta2_offset, d2, FK_a2, alpha2);
  Matrix4d T3 = createDHMatrix(angles.theta3 + theta3_offset, d3, FK_a3, alpha3);

  Matrix4d T4 = createDHMatrix(angles.theta4 + theta4_offset, d4, FK_a4, alpha4);
  Matrix4d T5 = createDHMatrix(angles.theta5 + theta5_offset, d5, FK_a5, alpha5);
  Matrix4d T6 = createDHMatrix(angles.theta6 + theta6_offset, d6, FK_a6, alpha6);

  Matrix4d Tn = T1 * T2 * T3 * T4 * T5 * T6;

  vector<Matrix4d> frames = {
      Matrix4d::Identity(),
      T1,
      T1 * T2,
      T1 * T2 * T3,
      T1 * T2 * T3 * T4,
      T1 * T2 * T3 * T4 * T5,
      Tn};

  Block position = Tn.block<3, 1>(0, 3); // origin of the frame Tn which corresponds to our arbitraty end effector

  // return pair(Coor(position(1), position(2), position(0)), frames);
  return pair(Coor(position(0), position(1), position(2)), frames);
}
