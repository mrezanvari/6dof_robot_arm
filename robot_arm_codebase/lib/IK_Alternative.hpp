/*
    This IK solver is based on the notes and diagrams provided in the PDF document.
    This solver was mainly focused on 3DoF but was extended to cover 6 DoF.
    The IKUtils.hpp is meant to strictly follow the textbook, its IK solver accounts for shoulder offset and multiple
    solutions which this one doesn't.
    This version also uses IKSoutionSet instead of IKSoution.
*/

#pragma once
#include <iostream>
using namespace std;

#include <math.h>
#include "FKUtils.hpp"

struct IKSoutionSet
{
    /*
        4 x arm solution branches
        2 x wrist solution branches
    */
    struct LeftArm
    {
        JointAngle elbowUp;
        JointAngle elbowDown;
    };

    struct RightArm
    {
        JointAngle elbowUp;
        JointAngle elbowDown;
    };

    struct Wrist
    {
        double theta1{};
        double theta2{};
        double theta3{};
    };

    LeftArm leftArm;
    RightArm rightArm;
    Wrist wrist; // wrist would have a pair of solutions that are 180 opposite of each other.
    // candidateSolution

    // JointAngle toJointAngle()
    // {
    //     // fill candidate solution and return
    // }
};

bool validateArmSolution(const Coor &newCoor, const IKSoutionSet &armSolutions, JointAngle *newMotorAngle, const vector<DHParams> &jointParams)
{
    ValidationFlags flags;

    flags.solution1_has_nan = isnan(armSolutions.rightArm.elbowUp.theta1) || isnan(armSolutions.rightArm.elbowUp.theta2) || isnan(armSolutions.rightArm.elbowUp.theta3);
    flags.solution2_has_nan = isnan(armSolutions.rightArm.elbowDown.theta1) || isnan(armSolutions.rightArm.elbowDown.theta2) || isnan(armSolutions.rightArm.elbowDown.theta3);

    *newMotorAngle = armSolutions.rightArm.elbowUp;

    if (flags.solution1_has_nan)
        *newMotorAngle = armSolutions.rightArm.elbowDown;

    flags.theta1_out_of_range = (newMotorAngle->theta1 > jointParams[0].limits.max) || (newMotorAngle->theta1 < jointParams[0].limits.min);
    flags.theta2_out_of_range = (newMotorAngle->theta2 > jointParams[1].limits.max) || (newMotorAngle->theta2 < jointParams[1].limits.min);
    flags.theta3_out_of_range = (newMotorAngle->theta3 > jointParams[2].limits.max) || (newMotorAngle->theta3 < jointParams[2].limits.min);

    flags.y_unreachable = newCoor.y < 0;

    uint8_t nan_mask = 0b11000000;
    uint8_t out_of_range_mask = 0b00111000;
    uint8_t pos_unreachable_mask = 0b00000111;

    bool has_nan = !flags.bits != !nan_mask;
    bool has_out_of_range = !(flags.bits & out_of_range_mask);
    bool has_invalid_coor = !(flags.bits & pos_unreachable_mask);

    return has_nan && has_out_of_range && has_invalid_coor;
}

bool validateWristSolution(const Orientation &newOrientation, const IKSoutionSet &wristSolution, JointAngle *newMotorAngle, const vector<DHParams> &jointParams)
{
    ValidationFlags flags;

    flags.solution1_has_nan = isnan(wristSolution.wrist.theta1) || isnan(wristSolution.wrist.theta2) || isnan(wristSolution.wrist.theta3);

    flags.theta1_out_of_range = (wristSolution.wrist.theta1 > jointParams[3].limits.max) || (wristSolution.wrist.theta1 < jointParams[3].limits.min);
    flags.theta2_out_of_range = (wristSolution.wrist.theta2 > jointParams[4].limits.max) || (wristSolution.wrist.theta2 < jointParams[4].limits.min);
    flags.theta3_out_of_range = (wristSolution.wrist.theta3 > jointParams[5].limits.max) || (wristSolution.wrist.theta3 < jointParams[5].limits.min);

    newMotorAngle->theta4 = wristSolution.wrist.theta1;
    newMotorAngle->theta5 = wristSolution.wrist.theta2;
    newMotorAngle->theta6 = wristSolution.wrist.theta3;

    uint8_t out_of_range_mask = 0b00111000;

    bool has_nan = !flags.solution1_has_nan;
    bool has_out_of_range = !(flags.bits & out_of_range_mask);

    return has_nan && has_out_of_range;
}

bool IK_Arm(const Coor &newpos, JointAngle *newMotorAngle, const vector<DHParams> &jointParams = globalJointParams)
{
    float r1, r2, r3, phi1, phi2, phi3;

    Coor localPos = newpos;
    if (newpos.axisType != Coor::CoorType::Y_UP)
        localPos = localPos.toYUp();

    IKSoutionSet armSolutions;

    double a1 = jointParams[0].d;
    double a2 = jointParams[2].a;
    double a3 = jointParams[3].d;

    r1 = sqrt(sq(localPos.x) + sq(localPos.z));
    r2 = localPos.y - a1;
    phi2 = atan2(r2, r1);
    r3 = sqrt(sq(r2) + sq(r1));
    phi1 = acos(((sq(a3) - sq(a2) - sq(r3))) / (-2 * a2 * r3));
    phi3 = acos((sq(r3) - sq(a2) - sq(a3)) / (-2 * a2 * a3));

    armSolutions.rightArm.elbowUp.theta1 = atan2(localPos.x, localPos.z);
    armSolutions.rightArm.elbowUp.theta2 = phi1 + phi2;
    armSolutions.rightArm.elbowUp.theta3 = phi3 - (M_PI / 2); // from the notes: M_PI - phi3 but the 0 of joint 3 in textbook is pointing straight down so -90

    armSolutions.rightArm.elbowDown.theta1 = armSolutions.rightArm.elbowUp.theta1;
    armSolutions.rightArm.elbowDown.theta2 = phi2 - phi1;
    armSolutions.rightArm.elbowDown.theta3 = -armSolutions.rightArm.elbowUp.theta3;

    // add left arm up and down solutions too
    armSolutions.leftArm.elbowUp = armSolutions.rightArm.elbowUp;     // for now ignore left arm
    armSolutions.leftArm.elbowDown = armSolutions.rightArm.elbowDown; // for now ignore left arm

    return validateArmSolution(localPos, armSolutions, newMotorAngle, jointParams);
}

bool IK(const Coor &newpos, Orientation &newOrientation, JointAngle *newMotorAngle, const vector<DHParams> &jointParams = globalJointParams)
{
    /*
      Chapter 5.2:
      Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). Robot modeling and control (Second edition.). John Wiley & Sons, Inc.
    */

    Coor localPos = newpos;
    if (newpos.axisType != Coor::CoorType::Z_UP)
        localPos = localPos.toZUp();

    Vector3d O = localPos.toVector3d(); // origin of desired end-effector position

    Matrix3d R = createRotationMatrix(newOrientation);         // rotation matrix of end-effector with respect to base origin
    Vector3d oc = O - (globalJointParams.back().d * R.col(2)); // offset origin of end-effector

    Coor ikIn(oc, Coor::CoorType::Z_UP);

    bool arm_may_proceed = IK_Arm(ikIn, newMotorAngle, jointParams);

    auto FK_out = FK(*newMotorAngle);
    vector<Matrix4d> frames = FK_out.second;

    Matrix4d H03 = frames[3];
    Matrix3d R03 = H03.block<3, 3>(0, 0);
    Matrix3d R30 = R03.inverse();
    Matrix3d R36 = R30 * R;

    IKSoutionSet wristSolution;
    wristSolution.wrist.theta1 = atan2(R36(2, 2), R36(0, 2));
    wristSolution.wrist.theta2 = atan2(sqrt(1 - sq(R36(1, 2))), -R36(1, 2));
    wristSolution.wrist.theta3 = atan2(-R36(2, 1), R36(2, 0));

    bool wrist_may_proceed = validateWristSolution(newOrientation, wristSolution, newMotorAngle, jointParams);

    return arm_may_proceed && wrist_may_proceed;
}