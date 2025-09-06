#pragma once
#include "RobotParams.hpp"

Matrix4d createDHMatrix(double theta, DHParams dhparams)
{
  theta += dhparams.theta_offset;
  double ct = cos(theta);
  double st = sin(theta);
  double ca = cos(dhparams.alpha);
  double sa = sin(dhparams.alpha);

  if (dhparams.isModifiedDH)
    return Matrix4d{
        {ct, -st, 0, dhparams.a},
        {st * ca, ct * ca, -sa, -dhparams.d * sa},
        {st * sa, ct * sa, ca, dhparams.d * ca},
        {0, 0, 0, 1}};

  return Matrix4d{
      {ct, -st * ca, st * sa, dhparams.a * ct},
      {st, ct * ca, -ct * sa, dhparams.a * st},
      {0, sa, ca, dhparams.d},
      {0, 0, 0, 1}};
}

Matrix3d createRotationMatrix(Orientation orientation)
{
  /*
    Chapter 2.5.1:
    Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). Robot modeling and control (Second edition.). John Wiley & Sons, Inc.
  */
  double phi = orientation.phi;
  double theta = orientation.theta;
  double psi = orientation.psi;

  Matrix3d R{{-sin(phi) * sin(psi) + cos(phi) * cos(psi) * cos(theta), -sin(phi) * cos(psi) - sin(psi) * cos(phi) * cos(theta), sin(theta) * cos(phi)},
             {sin(phi) * cos(psi) * cos(theta) + sin(psi) * cos(phi), -sin(phi) * sin(psi) * cos(theta) + cos(phi) * cos(psi), sin(phi) * sin(theta)},
             {-sin(theta) * cos(psi), sin(psi) * sin(theta), cos(theta)}};

  return R;
}

pair<Coor, vector<Matrix4d>> FK(const JointAngle &angles, const vector<DHParams> &jointParams = globalJointParams)
{
  vector<Matrix4d> frames(jointParams.size() + 1); // 1 frame per joint + 1 first identity frame for jacobian
  frames[0] = Matrix4d::Identity();

  for (size_t i = 0; i < jointParams.size(); ++i)
  {
    Matrix4d T = createDHMatrix(angles.thetas[i], jointParams[i]);
    frames[i + 1] = frames[i] * T;
  }

  Block position = frames.back().block<3, 1>(0, 3); // origin of the frame Tn which corresponds to our end effector

  // return pair(Coor(position(1), position(2), position(0)), frames);
  return pair(Coor(position(0), position(1), position(2)), frames);
}

Coor FK_precise(const JointAngle &angles)
{
  /*
    Created based on the paramters for the probe -> includes all offsets + centre of the joint 5 to the centre of the ball
    Used primarily for the PROBE mode
  */
  Coor position{140.54 * sin(angles.theta1) * sin(angles.theta4) * sin(angles.theta5) + 2.785 * sin(angles.theta1) * cos(angles.theta4) + 19.508 * sin(angles.theta1) - 2.785 * sin(angles.theta4) * cos(angles.theta1) * cos(angles.theta2 + angles.theta3) + 140.54 * sin(angles.theta5) * cos(angles.theta1) * cos(angles.theta4) * cos(angles.theta2 + angles.theta3) + 140.54 * sin(angles.theta2 + angles.theta3) * cos(angles.theta1) * cos(angles.theta5) + 231.03 * sin(angles.theta2 + angles.theta3) * cos(angles.theta1) + 250.201 * cos(angles.theta1) * cos(angles.theta2),
                -2.785 * sin(angles.theta1) * sin(angles.theta4) * cos(angles.theta2 + angles.theta3) + 140.54 * sin(angles.theta1) * sin(angles.theta5) * cos(angles.theta4) * cos(angles.theta2 + angles.theta3) + 140.54 * sin(angles.theta1) * sin(angles.theta2 + angles.theta3) * cos(angles.theta5) + 231.03 * sin(angles.theta1) * sin(angles.theta2 + angles.theta3) + 250.201 * sin(angles.theta1) * cos(angles.theta2) - 140.54 * sin(angles.theta4) * sin(angles.theta5) * cos(angles.theta1) - 2.785 * cos(angles.theta1) * cos(angles.theta4) - 19.508 * cos(angles.theta1),
                250.201 * sin(angles.theta2) - 2.785 * sin(angles.theta4) * sin(angles.theta2 + angles.theta3) + 140.54 * sin(angles.theta5) * sin(angles.theta2 + angles.theta3) * cos(angles.theta4) - 140.54 * cos(angles.theta5) * cos(angles.theta2 + angles.theta3) - 231.03 * cos(angles.theta2 + angles.theta3) + 167.719};

  return position;
}