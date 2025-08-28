#pragma once
#include <iostream>
using namespace std;

#include <math.h>
#include "FKUtils.hpp"

const double IK_a1 = 167.719; // height of lower joint
const double IK_a2 = 250.201; // length of lower arm
const double IK_a3 = 231.03;  // length of upper arm -> centre of joint 5

const double theta1_max = rad(180);
const double theta1_min = rad(0);

const double theta2_max = rad(180);
const double theta2_min = rad(0);

const double theta3_max = rad(130);
const double theta3_min = rad(-130);

const double theta4_max = rad(360);
const double theta4_min = rad(0);

const double theta5_max = rad(100);
const double theta5_min = rad(-100);

const double theta6_max = rad(360);
const double theta6_min = rad(-360);

bool validateArmSolution(const Coor &newCoor, const IKSoutionSet &armSolutions, JointAngle *newMotorAngle)
{
  ValidationFlags flags;

  flags.solution1_has_nan = isnan(armSolutions.rightArm.elbowUp.theta1) || isnan(armSolutions.rightArm.elbowUp.theta2) || isnan(armSolutions.rightArm.elbowUp.theta3);
  flags.solution2_has_nan = isnan(armSolutions.rightArm.elbowDown.theta1) || isnan(armSolutions.rightArm.elbowDown.theta2) || isnan(armSolutions.rightArm.elbowDown.theta3);

  *newMotorAngle = armSolutions.rightArm.elbowUp;

  if (flags.solution1_has_nan)
    *newMotorAngle = armSolutions.rightArm.elbowDown;

  flags.theta1_out_of_range = (theta1_max < newMotorAngle->theta1) || (newMotorAngle->theta1 < theta1_min);
  flags.theta2_out_of_range = (theta2_max < newMotorAngle->theta2) || (newMotorAngle->theta2 < theta2_min);
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

bool validateWristSolution(const Orientation &newOrientation, const IKSoutionSet &wristSolution, JointAngle *newMotorAngle)
{
  ValidationFlags flags;

  flags.solution1_has_nan = isnan(wristSolution.wrist.theta1) || isnan(wristSolution.wrist.theta2) || isnan(wristSolution.wrist.theta3);

  flags.theta1_out_of_range = (wristSolution.wrist.theta1 > theta4_max) || (wristSolution.wrist.theta1 < theta4_min);
  flags.theta2_out_of_range = (wristSolution.wrist.theta2 > theta5_max) || (wristSolution.wrist.theta2 < theta5_min);
  flags.theta3_out_of_range = (wristSolution.wrist.theta3 > theta6_max) || (wristSolution.wrist.theta3 < theta6_min);

  newMotorAngle->theta4 = wristSolution.wrist.theta1;
  newMotorAngle->theta5 = wristSolution.wrist.theta2;
  newMotorAngle->theta6 = wristSolution.wrist.theta3;

  uint8_t out_of_range_mask = 0b00111000;

  bool has_nan = !flags.solution1_has_nan;
  bool has_out_of_range = !(flags.bits & out_of_range_mask);

  return has_nan && has_out_of_range;
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
  armSolutions.rightArm.elbowUp.theta3 = phi3 - (M_PI / 2); // from the notes: M_PI - phi3 but the 0 of joint 3 in textbook is pointing straight down so -90

  armSolutions.rightArm.elbowDown.theta1 = armSolutions.rightArm.elbowUp.theta1;
  armSolutions.rightArm.elbowDown.theta2 = phi2 - phi1;
  armSolutions.rightArm.elbowDown.theta3 = -armSolutions.rightArm.elbowUp.theta3;

  // add left arm up and down solutions too
  armSolutions.leftArm.elbowUp = armSolutions.rightArm.elbowUp;     // for now ignore left arm
  armSolutions.leftArm.elbowDown = armSolutions.rightArm.elbowDown; // for now ignore left arm

  return validateArmSolution(newpos, armSolutions, newMotorAngle);
}

bool IK(const Coor newpos, Orientation newOrientation, JointAngle *newMotorAngle)
{
  Vector3d O{{
      newpos.z,
      newpos.x,
      newpos.y,
  }};

  Matrix3d R = createRotationMatrix(newOrientation);
  Vector3d oc = O - (globalJointParams.back().d * R.col(2));

  Coor ikIn;
  ikIn.x = oc(1);
  ikIn.y = oc(2);
  ikIn.z = oc(0);

  bool arm_may_proceed = IK_Arm(ikIn, newMotorAngle);

  auto FK_out = FK(*newMotorAngle);
  vector<Matrix4d> frames = FK_out.second;

  Matrix4d H03 = frames[3];
  Matrix3d R03 = H03.block<3, 3>(0, 0);
  Matrix3d R30 = R03.inverse();
  Matrix3d R36 = R30 * R;

  IKSoutionSet wristSolution;
  wristSolution.wrist.theta1 = atan2(R36(2, 2), R36(0, 2));
  wristSolution.wrist.theta2 = atan2(sqrt(1 - pow(R36(1, 2), 2)), -R36(1, 2));
  wristSolution.wrist.theta3 = atan2(-R36(2, 1), R36(2, 0));

  bool wrist_may_proceed = validateWristSolution(newOrientation, wristSolution, newMotorAngle);

  return arm_may_proceed && wrist_may_proceed;
  return true;
}