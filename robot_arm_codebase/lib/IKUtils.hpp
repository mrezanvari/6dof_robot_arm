#include <iostream>
using namespace std;

#include <math.h>

const double IK_a1 = 167.719;         // height of lower joint
const double IK_a2 = 250.201;         // length of lower arm
const double IK_a3 = 231.03 + 140.54; // length of upper arm -> center of middle +  safe offset

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

bool validateArmSolution(const Coor &newCoor, const IKSoutionSet &armSolutions, JointAngle *newMotorAngle)
{
  ValidationFlags flags;

  flags.solution1_has_nan = isnan(armSolutions.rightArm.elbowUp.theta1) || isnan(armSolutions.rightArm.elbowUp.theta2) || isnan(armSolutions.rightArm.elbowUp.theta3);
  flags.solution2_has_nan = isnan(armSolutions.rightArm.elbowDown.theta1) || isnan(armSolutions.rightArm.elbowDown.theta2) || isnan(armSolutions.rightArm.elbowDown.theta3);

  *newMotorAngle = armSolutions.rightArm.elbowUp;

  if (flags.solution1_has_nan)
    *newMotorAngle = armSolutions.rightArm.elbowDown;

  // add left arm up and down validation too

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

bool validateWristSolution(const Coor &newCoor, const IKSoutionSet &wristSolutions, JointAngle *newMotorAngle)
{
  // do validation on wrist
  return true;
}

bool IK_Arm(const Coor newpos, JointAngle *newMotorAngle)
{
  float r1, r2, r3, phi1, phi2, phi3;

  IKSoutionSet armSolutions;

  r1 = sqrt(pow(newpos.x, 2) + pow(newpos.z, 2));
  r2 = newpos.y - IK_a1;
  phi2 = atan2(r2, r1);
  r3 = sqrt(pow(r2, 2) + pow(r1, 2));
  phi1 = acos(((pow(IK_a3, 2) - pow(IK_a2, 2) - pow(r3, 2))) / (-2 * IK_a2 * r3));
  phi3 = acos((pow(r3, 2) - pow(IK_a2, 2) - pow(IK_a3, 2)) / (-2 * IK_a2 * IK_a3));

  armSolutions.rightArm.elbowUp.theta1 = atan2(newpos.x, newpos.z);
  armSolutions.rightArm.elbowUp.theta2 = phi1 + phi2;
  armSolutions.rightArm.elbowUp.theta3 = M_PI - phi3;

  armSolutions.rightArm.elbowDown.theta1 = armSolutions.rightArm.elbowUp.theta1;
  armSolutions.rightArm.elbowDown.theta2 = phi2 - phi1;
  armSolutions.rightArm.elbowDown.theta3 = -armSolutions.rightArm.elbowUp.theta3;

  // add left arm up and down solutions too

  return validateArmSolution(newpos, armSolutions, newMotorAngle);
}

bool IK_Wrist(const Coor newpos, JointAngle *newMotorAngle)
{

  // do IK for wrist;
  IKSoutionSet wristSolutions;
  return validateWristSolution(newpos, wristSolutions, newMotorAngle);
}

bool IK(const Coor newpos, JointAngle *newMotorAngle)
{
  bool wrist_out = IK_Wrist(newpos, newMotorAngle);
  bool arm_out = IK_Arm(newpos, newMotorAngle);

  return arm_out && wrist_out;
}