#include <iostream>
using namespace std;

#include <math.h>

const float IK_a1 = 167.719;                          // height of lower joint
const float IK_a2 = 250.201;                          // length of lower arm
const float IK_a3 = 177.485 + 54.70 + 95.069 + 58.60; // length of upper arm -> center of middle +  safe offset

const double theta1_max = rad(0);
const double theta1_min = rad(-180);

const double theta2_max = rad(0);
const double theta2_min = rad(-180);

const double theta3_max = rad(130);
const double theta3_min = rad(-130);

const double theta4_max = rad(0);
const double theta4_min = rad(360);

const double theta5_max = rad(130);
const double theta5_min = rad(-130);

const double theta6_max = rad(0);
const double theta6_min = rad(360);

bool validateSolution(const Coor &newCoor, const pair<JointAngle, JointAngle> &solutions, JointAngle *newMotorAngle)
{
  ValidationFlags flags;

  flags.solution1_has_nan = isnan(solutions.first.theta1) || isnan(solutions.first.theta2) || isnan(solutions.first.theta3);
  flags.solution2_has_nan = isnan(solutions.second.theta1) || isnan(solutions.second.theta2) || isnan(solutions.second.theta3);

  *newMotorAngle = solutions.first;

  if (flags.solution1_has_nan)
    *newMotorAngle = solutions.second;

  flags.theta1_out_of_range = (theta1_max < -newMotorAngle->theta1) || (-newMotorAngle->theta1 < theta1_min); // "-" because ccw is positive angle
  flags.theta2_out_of_range = (theta2_max < -newMotorAngle->theta2) || (-newMotorAngle->theta2 < theta2_min); // "-" because ccw is positive angle
  flags.theta3_out_of_range = (theta3_max < newMotorAngle->theta3) || (newMotorAngle->theta3 < theta3_min);

  flags.y_unreachable = newCoor.y < 0;

  uint8_t nan_mask = 0b11000000;
  uint8_t out_of_range_mask = 0b00111000;
  uint8_t pos_unreachable_mask = 0b00000111;

  bool has_nan = !flags.bits != !nan_mask;
  bool has_out_of_range = !(flags.bits & out_of_range_mask);
  bool has_invalid_coor = !(flags.bits & pos_unreachable_mask);

  return has_nan && has_out_of_range && has_invalid_coor;
}

bool IK(const Coor newpos, JointAngle *newMotorAngle)
{
  float r1, r2, r3, phi1, phi2, phi3;

  pair<JointAngle, JointAngle> solutions;

  r1 = sqrt(pow(newpos.x, 2) + pow(newpos.z, 2));
  r2 = newpos.y - IK_a1;
  phi2 = atan(r2 / r1); // should this be atan2?
  r3 = sqrt(pow(r2, 2) + pow(r1, 2));
  phi1 = acos(((pow(IK_a3, 2) - pow(IK_a2, 2) - pow(r3, 2))) / (-2 * IK_a2 * r3));
  phi3 = acos((pow(r3, 2) - pow(IK_a2, 2) - pow(IK_a3, 2)) / (-2 * IK_a2 * IK_a3));

  solutions.first.theta1 = atan2(newpos.x, newpos.z);
  solutions.first.theta2 = phi1 + phi2;
  solutions.first.theta3 = M_PI - phi3;

  solutions.second.theta1 = solutions.first.theta1;
  solutions.second.theta2 = phi2 - phi1;
  solutions.second.theta3 = -solutions.first.theta3;

  return validateSolution(newpos, solutions, newMotorAngle);
}