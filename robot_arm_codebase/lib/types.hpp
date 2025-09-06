#pragma once
#ifdef ARDUINO
#include <ArduinoEigenDense.h>
#else
#include <Eigen/Dense>
#endif
#include <iostream>

using namespace Eigen;
using Eigen::MatrixXd;

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
    enum CoorType
    {
        Z_UP,
        Y_UP,
    };

    enum CoorScale
    {
        MILLIMETER,
        METER,
    };

    double x{};
    double y{};
    double z{};
    CoorType axisType = CoorType::Y_UP;
    CoorScale coorScale = CoorScale::MILLIMETER;

    Coor() = default;
    Coor(double x, double y, double z, CoorType axisType = CoorType::Y_UP, CoorScale coorScale = CoorScale::MILLIMETER) : x(x),
                                                                                                                          y(y),
                                                                                                                          z(z),
                                                                                                                          axisType(axisType),
                                                                                                                          coorScale(coorScale) {}
    explicit Coor(const Vector3d &v, CoorType axisType, CoorScale coorScale) : x(v(0)),
                                                                               y(v(1)),
                                                                               z(v(2)),
                                                                               axisType(axisType),
                                                                               coorScale(coorScale) {} // conversion from vector3d

    Coor toZUp() const
    {
        // to convert from human-readable Y-Up to the book's Z-Up coordinates
        return Coor(z, x, y, Z_UP);
    }

    Coor toYUp() const
    {
        // to convert from book's Z-Up to human-readable Y-Up coordinates
        return Coor(y, z, x, Y_UP);
    }

    Vector3d toVector3d() const
    {
        return Vector3d{{
            x,
            y,
            z,
        }};
    }

    Coor toMeter()
    {
        return Coor(x / 1000.0,
                    y / 1000.0,
                    z / 1000.0,
                    axisType,
                    CoorScale::METER);
    }

    Coor toMillimeters()
    {
        return Coor(x * 1000.0,
                    y * 1000.0,
                    z * 1000.0,
                    axisType,
                    CoorScale::MILLIMETER);
    }
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
                         mot_r(theta3),
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

    struct AngleLimit
    {
        double min{0};
        double max{0};

        AngleLimit() = default;
        AngleLimit(double min, double max) : min(min), max(max) {}
    };

    double d{};
    double a{};
    double alpha{};
    double theta_offset{};
    bool isModifiedDH = true; // -> by default it is modified DH
    AngleLimit limits;

    DHParams() = default;
    DHParams(double d, double a, double alpha,
             double theta_offset,
             AngleLimit limits = {0, 0},
             bool isModifiedDH = true) : d(d),
                                         a(a),
                                         alpha(alpha),
                                         theta_offset(theta_offset),
                                         limits(limits),
                                         isModifiedDH(isModifiedDH) {}
};

#pragma pack(push, 1)
struct IKSolution
{
    /*
        This struct is meant to hold all 8 solutions of the IK
        2: elbow up/down
        2: wrist up/down
        2: shoulder left/right

        all thetas are accessible as a flat array of 36 thetas
    */

    struct Wrist
    {
        double theta1;
        double theta2;
        double theta3;
    };

    struct Elbow
    {
        double theta1;
        double theta2;
        double theta3;
        Wrist wrist1;
        Wrist wrist2;
    };

    struct Arm
    {
        Elbow up;
        Elbow down;
    };

    union
    {
        struct
        {
            Arm right;
            Arm left;
        };
        double thetas[36];
    };

    union ValidationFlags
    {
        struct
        {
            bool solution1_is_valid : 1;
            bool solution2_is_valid : 1;
            bool solution3_is_valid : 1;
            bool solution4_is_valid : 1;
            bool solution5_is_valid : 1;
            bool solution6_is_valid : 1;
            bool solution7_is_valid : 1;
            bool solution8_is_valid : 1;
        };
        uint8_t bits;

        constexpr ValidationFlags() : bits(0) {}
    } validationFlags;

    IKSolution() // had to manually zero everything to prevent weird values
    {
        for (double &theta : thetas)
            theta = 0; // NAN; // now that this default value can be set to anything, maybe nan would be better?
    }
};
#pragma pack(pop)

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

struct Orientation
{
    double phi{};
    double theta{};
    double psi{};

    Orientation() = default;
    Orientation(double phi, double theta, double psi) : phi(phi),
                                                        theta(theta),
                                                        psi(psi) {}
};