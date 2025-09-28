#pragma once
#include <iostream>
using namespace std;

#include <math.h>
#include "FKUtils.hpp"

#define sq(x) ((x) * (x)) // Sqare function instead of pow(x, 2)

double normalizeAngle(double radAngle)
{
  // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
  while (radAngle >= M_PI)
    radAngle -= (2.0 * M_PI);
  while (radAngle < -M_PI)
    radAngle += (2.0 * M_PI);

  return radAngle;
}

void validate3DoFIKSolutions(IKSolution *newIKSolution, const vector<DHParams> &jointParams = globalJointParams)
{
  /*
    Validate 3 dof solutions
    At this stage, any nan values would automatically invalidate both wrist options,
    hence the 3*isnan -> 3 in bin = 11.
  */
  const size_t solutionThetaCount = 36;
  const size_t nexSolutionOffset = 9;

  uint8_t currentSolution = 0;
  for (uint8_t offset = 0; offset < solutionThetaCount; offset += nexSolutionOffset)
  {
    for (uint8_t i = offset; i < offset + 3; ++i)
      newIKSolution->validationFlags.bits |= 3 * ((isnan(newIKSolution->thetas[i])) ||
                                                  (newIKSolution->thetas[i] < jointParams[i - offset].limits.min ||
                                                   newIKSolution->thetas[i] > jointParams[i - offset].limits.max))
                                             << currentSolution; // shift 11 or 00 to position of current solution then OR with current state

    currentSolution += 2;
  }

  newIKSolution->validationFlags.bits = ~newIKSolution->validationFlags.bits; // so far we assign 1 to ones that are invalid -> so invert to make sense
}

void solve3DoFIK(const Coor &newpos, IKSolution *newIKSolution, const vector<DHParams> &jointParams = globalJointParams)
{
  Coor localPos = newpos;
  if (newpos.axisType != Coor::CoorType::Z_UP)
    localPos = localPos.toZUp();
  // if (newpos.coorScale != Coor::CoorScale::METER)
  //   localPos = localPos.toMeter();

  if (localPos.z < 0)
  {
    for (double &theta : newIKSolution->thetas)
      theta = NAN;
    return;
  }

  double d1 = jointParams[0].d; // length of lower arm -> book uses DH, we use MDH so here we use d rather than a
  double a2 = jointParams[2].a; // length of lower arm
  double a3 = jointParams[3].d; // length of upper arm
  double d = jointParams[1].d;  // shoulder offset -> from the second joint DH d parameter

  double phi = atan2(localPos.y, localPos.x);
  double r2 = sq(localPos.y) + sq(localPos.x) - sq(d);       // r squared -> from equation 5.24 and 5.26
  double s = (localPos.z - d1);                              // from equation 5.24 and 5.26
  double D = (r2 + sq(s) - sq(a2) - sq(a3)) / (2 * a2 * a3); // from equation 5.24

  double r2_sqrt = sqrt(r2);
  double alpha_right = atan2(d, r2_sqrt);
  double alpha_left = atan2(-d, -r2_sqrt);

  double theta1_right = phi - alpha_right; // right arm -> for us
  double theta1_left = phi + alpha_left;   // left arm

  // NOTE: Our quadrants are different from the book
  double acos_sqrt = sqrt(1 - sq(D));
  double atan2_s_r2_sqrt = atan2(s, r2_sqrt);

  double theta3_elbow_up = atan2(D, acos_sqrt);
  double theta2_elbow_up = atan2_s_r2_sqrt + atan2(a3 * cos(theta3_elbow_up), a2 + (a3 * sin(theta3_elbow_up)));

  double theta3_elbow_down = atan2(D, -acos_sqrt);
  double theta2_elbow_down = atan2_s_r2_sqrt + atan2(a3 * cos(theta3_elbow_down), a2 + (a3 * sin(theta3_elbow_down)));

  newIKSolution->right.up.theta1 = theta1_right;
  newIKSolution->right.up.theta2 = theta2_elbow_up;
  newIKSolution->right.up.theta3 = theta3_elbow_up;

  newIKSolution->right.down.theta1 = theta1_right;
  newIKSolution->right.down.theta2 = theta2_elbow_down;
  newIKSolution->right.down.theta3 = theta3_elbow_down;

  newIKSolution->left.up.theta1 = theta1_left;
  newIKSolution->left.up.theta2 = M_PI - theta2_elbow_up;
  newIKSolution->left.up.theta3 = M_PI - theta3_elbow_up;

  newIKSolution->left.down.theta1 = theta1_left;
  newIKSolution->left.down.theta2 = M_PI - theta2_elbow_down;
  newIKSolution->left.down.theta3 = M_PI - theta3_elbow_down;
}

void validateWristIKSolutions(IKSolution *newIKSolution, const vector<DHParams> &jointParams = globalJointParams)
{
  int offsetMultiplier = 1; // value will be [1, 2, 4, 5, 7, 8, 10, 11]
  for (int currentSolution = 0; currentSolution < 8; ++currentSolution)
  {
    int offset = offsetMultiplier * 3;
    offsetMultiplier += ((currentSolution + 1) % 2 == 0) ? 2 : 1; // skip 3, 6, 9

    if (!(newIKSolution->validationFlags.bits & (1 << currentSolution)))
      continue;

    for (int i = offset; i < offset + 3; ++i)
      newIKSolution->validationFlags.bits &= ~((isnan(newIKSolution->thetas[i]) ||
                                                (newIKSolution->thetas[i] < jointParams[i - offset + 3].limits.min ||
                                                 newIKSolution->thetas[i] > jointParams[i - offset + 3].limits.max))
                                               << currentSolution); // shift invalid solution to position in byte then invert and AND with validation flags
  }
}

IKSolution solveFullIK(const Coor &newpos, Orientation &newOrientation, JointAngle *newMotorAngle, const vector<DHParams> &jointParams = globalJointParams)
{
  Coor localPos = newpos;
  if (newpos.axisType != Coor::CoorType::Z_UP)
    localPos = localPos.toZUp();
  // if (newpos.coorScale != Coor::CoorScale::METER)
  //   localPos = localPos.toMeter();

  Vector3d O = localPos.toVector3d(); // origin of desired end-effector position

  Matrix3d R = createRotationMatrix(newOrientation);         // rotation matrix of end-effector with respect to base origin
  Vector3d oc = O - (globalJointParams.back().d * R.col(2)); // offset origin of end-effector

  Coor ikIn(oc, Coor::CoorType::Z_UP, Coor::CoorScale::METER);

  IKSolution newIKSolution;
  solve3DoFIK(ikIn, &newIKSolution, jointParams);
  validate3DoFIKSolutions(&newIKSolution);

  /*
    There are 4 arm solutions -> left up, left down, right up, right down
    Each have 2 wrist solutions -> left up wrist1, left down wrist2 ...

    here, we traverse the thetas of IKSolution by the number of arm solutions = 4
    we check the validationFlags for arm solutions that are valid
    we then, solve for the orientation of each and populate both wrist solutions
    which are the last 6 thetas of the solution.
    The indicies are formated as:
    index:  012 ........... 345 ....... 678
        (arm thetas)      (wrist 1)   (wrist2)
  */

  for (uint8_t solution = 0; solution < 4; ++solution)
  {
    uint8_t offset = solution * 9;
    if (newIKSolution.validationFlags.bits & (1 << solution * 2)) // check for valid arm
    {
      JointAngle tempAngles;
      for (uint8_t i = 0; i < 3; ++i) // convert to JointAngles
        tempAngles.thetas[i] = newIKSolution.thetas[i + offset];

      auto FK_out = FK(tempAngles);
      vector<Matrix4d> frames = FK_out.second;

      Matrix4d H03 = frames[3];
      Matrix3d R03 = H03.block<3, 3>(0, 0);
      Matrix3d R30 = R03.inverse();
      Matrix3d R36 = R30 * R;

      /*
        Symbolic matrix of R36:

        ⎡-sin(t₄)⋅sin(t₆) + cos(t₄)⋅cos(t₅)⋅cos(t₆)  -sin(t₄)⋅cos(t₆) - sin(t₆)⋅cos(t₄)⋅cos(t₅)  sin(t₅)⋅cos(t₄)⎤
        ⎢                                                                                                       ⎥
        ⎢             sin(t₅)⋅cos(t₆)                             -sin(t₅)⋅sin(t₆)                  -cos(t₅)    ⎥
        ⎢                                                                                                       ⎥
        ⎣sin(t₄)⋅cos(t₅)⋅cos(t₆) + sin(t₆)⋅cos(t₄)   -sin(t₄)⋅sin(t₆)⋅cos(t₅) + cos(t₄)⋅cos(t₆)  sin(t₄)⋅sin(t₅)⎦
      */

      // wrist 1 solutions

      double theta1 = atan2(R36(2, 2), R36(0, 2));
      double theta2 = atan2(sqrt(1 - sq(R36(1, 2))), -R36(1, 2));
      double theta3 = atan2(-R36(1, 1), R36(1, 0));

      // wrist 2 solutions
      double theta1_2 = atan2(R36(2, 2), -R36(0, 2));
      double theta2_2 = -theta2;
      double theta3_2 = atan2(-R36(1, 1), -R36(1, 0));

      // offset + 3 arm thetas + index of wrist solution
      newIKSolution.thetas[offset + 3 + 0] = normalizeAngle(theta1);
      newIKSolution.thetas[offset + 3 + 1] = normalizeAngle(theta2);
      newIKSolution.thetas[offset + 3 + 2] = normalizeAngle(theta3);
      newIKSolution.thetas[offset + 3 + 3] = normalizeAngle(theta1_2);
      newIKSolution.thetas[offset + 3 + 4] = normalizeAngle(theta2_2);
      newIKSolution.thetas[offset + 3 + 5] = normalizeAngle(theta3_2);
    }
  }

  validateWristIKSolutions(&newIKSolution);

  if (newIKSolution.validationFlags.solution1_is_valid)
    for (uint8_t i = 0; i < 6; ++i) // simply set the first one for output
      newMotorAngle->thetas[i] = newIKSolution.thetas[i];
  else
    for (double &theta : newMotorAngle->thetas)
      theta = NAN;

  return newIKSolution;
}

int pickBestSolution(const IKSolution &newIKSolution, const JointAngle &lastStableSolution, JointAngle *bestSolution)
{
  // This function may not produce the expected result please avoid using.
  int offsetMultiplier = 1; // value will be [1, 2, 4, 5, 7, 8, 10, 11]

  double delta = INT_MAX;
  int chosen = -1;

  for (int currentSolution = 0; currentSolution < 8; ++currentSolution)
  {
    int offset = offsetMultiplier * 3;
    offsetMultiplier += ((currentSolution + 1) % 2 == 0) ? 2 : 1; // skip 3, 6, 9

    if (!(newIKSolution.validationFlags.bits & (1 << currentSolution)))
      continue;

    JointAngle thisSolution;
    int armOffset = floor(currentSolution / 2.0);

    for (int i = 0; i < 3; ++i)
      thisSolution.thetas[i] = newIKSolution.thetas[armOffset + i];

    for (int i = 0; i < 3; ++i)
      thisSolution.thetas[i + 3] = newIKSolution.thetas[offset + i];

    double thisDelta = 0;
    for (int i = 0; i < 6; ++i)
      thisDelta += abs(lastStableSolution.thetas[i] - thisSolution.thetas[i]);

    if (thisDelta < delta)
    {
      *bestSolution = thisSolution;
      chosen = currentSolution;
      delta = thisDelta;
    }
  }

  return chosen;
}