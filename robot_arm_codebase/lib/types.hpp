#include <iostream>

using namespace std;

const double deg2radMult = (M_PI / 180.0);          // degrees to radians
const double rad2degMult = (180.0 / M_PI);          // radians to degrees
const double mot2angMult = (360 / 9);               // motor position to angle -> 1 motor = 9 rev of driver -> 1:9 gear
const double wmot2angMult = (360 / 8);              // motor position to angle -> 1 motor = 9 rev of driver -> 1:9 gear
const double ang2motMult = (1 / mot2angMult);       // angle to motor position
const double ang2wmotMult = (1 / wmot2angMult);     // angle to motor position
const double rad2motMult = (1 / ((M_PI * 2) / 9));  // radians to motor position
const double rad2wmotMult = (1 / ((M_PI * 2) / 8)); // radians to motor position

#define rad(deg) ((deg) * deg2radMult)
#define deg(rad) ((rad) * rad2degMult)
#define mot_a(ang) ((ang) * rad2degMult * ang2motMult * deg2radMult)   // angle to motor postion
#define mot_r(rad) ((rad) * rad2motMult)                               // rad to motor postion
#define mpos2rad(pos) ((pos) / rad2motMult)                            // motor position to rad
#define wmot_a(ang) ((ang) * rad2degMult * ang2wmotMult * deg2radMult) // angle to motor postion
#define wmot_r(rad) ((rad) * rad2wmotMult)                             // rad to motor postion
#define wmpos2rad(pos) ((pos) / rad2wmotMult)                          // motor position to rad

struct JointAngle;
struct MotorPosition;

struct Coor
{
    double x{};
    double y{};
    double z{};

    Coor() = default;
    Coor(double x, double y, double z) : x(x), y(y), z(z) {}
};

struct MotorPosition
{
    double pos1{};
    double pos2{};
    double pos3{};

    double pos4{};
    double pos5{};
    double pos6{};

    MotorPosition() = default;
    MotorPosition(double pos1, double pos2, double pos3,
                  double pos4, double pos5, double pos6) : pos1(pos1),
                                                           pos2(pos2),
                                                           pos3(pos3),
                                                           pos4(pos4),
                                                           pos5(pos5),
                                                           pos6(pos6) {}

    JointAngle toJointAngle();
};

struct JointAngle
{
    union
    {
        struct
        {
            double theta1;
            double theta2;
            double theta3;
            double theta4;
            double theta5;
            double theta6;
        };
        double thetas[6];
    };

    JointAngle() = default;

    JointAngle(double theta1, double theta2, double theta3,
               double theta4, double theta5, double theta6) : theta1(theta1),
                                                              theta2(theta2),
                                                              theta3(theta3),
                                                              theta4(theta4),
                                                              theta5(theta5),
                                                              theta6(theta6) {}

    MotorPosition toMotorPosition();
};

JointAngle MotorPosition::toJointAngle()
{
    return JointAngle(mpos2rad(pos1),
                      mpos2rad(pos2),
                      mpos2rad(pos3),
                      wmpos2rad(pos4),
                      wmpos2rad(pos5),
                      wmpos2rad(pos6));
}

MotorPosition JointAngle::toMotorPosition()
{
    return MotorPosition(mot_r(theta1),
                         mot_r(theta2),
                         mot_r(((M_PI / 2) - theta3)), // best to make it so it requires no change but for now...
                         wmot_r(theta4),
                         wmot_r(theta5),
                         wmot_r(theta6));
}

union ValidationFlags
{
    struct
    {
        bool solution1_has_nan : 1;
        bool solution2_has_nan : 1;
        bool theta1_out_of_range : 1;
        bool theta2_out_of_range : 1;
        bool theta3_out_of_range : 1;
        bool y_unreachable : 1;
        bool pad5 : 1;
        bool pad6 : 1;
    };
    uint8_t bits{};
};

struct DHParams
{
    /*
        DH/MDH parameters struct
        For DH:
        a = distance along xi from the intersection of the xi and zi − 1 axes to .
        d = distance along zi − 1 from to the intersection of the xi and zi − 1 axes. If joint i is prismatic, di is variable.

        For MDH:
        a = distance along xi - 1 from the intersection of the xi and zi − 1 axes to .
        d = distance along zi from to the intersection of the xi and zi − 1 axes. If joint i is prismatic, di is variable.

        alpha = the angle from zi − 1 to zi measured about xi.
        θi = the angle from xi − 1 to xi measured about zi − 1. If joint i is revolute, θi is variable

        Chapter 3.2, fig 3.10:
        Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). Robot modeling and control (Second edition.). John Wiley & Sons, Inc.
        https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    */
    double d{};
    double a{};
    double alpha{};
    double theta_offset{};
    bool isModifiedDH = true; // -> by default it is modified DH
    pair<double, double> limits{numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN()};

    DHParams() = default;
    DHParams(double d, double a, double alpha,
             double theta_offset,
             pair<double, double> limits = {numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN()},
             bool isModifiedDH = true) : d(d),
                                         a(a),
                                         alpha(alpha),
                                         theta_offset(theta_offset),
                                         limits(limits),
                                         isModifiedDH(isModifiedDH) {}
};

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

    LeftArm leftArm;
    RightArm rightArm;
    pair<JointAngle, JointAngle> wrist;
};