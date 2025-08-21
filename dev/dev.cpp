#include <iostream>
#include "../robot_arm_codebase/lib/types.hpp"
#include "../robot_arm_codebase/lib/IKUtils.hpp"
#include "../robot_arm_codebase/lib/FKUtils.hpp"
#include "../robot_arm_codebase/lib/JacobianUtils.hpp"
#include <vector>

Coor globalUserPos;
JointAngle tempAngle;

using namespace Eigen;
using Eigen::MatrixXd;

double thresh = 1e-5;

template <typename Derived>
void print_mat(const MatrixBase<Derived> &mat, bool newline = true, string formatstr = "  % 16.10G│")
{
    if (newline)
        cout << endl;
    for (size_t x = 0; x < mat.rows(); ++x)
    {
        for (size_t y = 0; y < mat.cols(); ++y)
            printf(formatstr.c_str(), mat(x, y));
        if (newline)
            cout << endl;
    }
    if (newline)
        cout << endl;
}

int main()
{

    /*
     */

    globalUserPos.x = 280;
    globalUserPos.y = 100;
    globalUserPos.z = -200;

    // printf("Coor->         x= %1.2f y= %1.2f z= %1.2f\r\n", globalUserPos.x, globalUserPos.y, globalUserPos.z);
    bool mayProceed = IK(globalUserPos, &tempAngle);

    // MotorPosition tempPos = tempAngle.toMotorPosition();
    printf("Out->          θ1=%1.3f θ2=%1.3f θ3=%1.3f\r\n\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));
    // printf("Out->          θ1=%1.3f θ2=%1.3f θ3=%1.3f\r\n\r\n", tempAngle.theta1, tempAngle.theta2, tempAngle.theta3);
    if (!mayProceed)
        cout << "You shall not pass!" << endl
             << endl;
    else
        cout << "Proceed!" << endl
             << endl;

    MotorPosition motorAngles;

    // motorAngles.pos3 = mot_a(90);

    // θ1: 0.093 θ2: -0.168 θ3: 0.053 θ4: 0.060 θ5: 0.013 θ6: 0.042

    motorAngles.pos1 = -1.645;
    motorAngles.pos2 = 0.2;
    motorAngles.pos3 = -2.5;
    motorAngles.pos4 = 0.004;
    motorAngles.pos5 = 0.005;
    motorAngles.pos6 = 20;

    /// vFK x: 435.308 y: 10.458 z:-67.057 │ theta1: -2.469 theta2: -0.748 theta3: 2.038 theta4: -2.161 theta5: -4.345 theta6: 0.000

    // JointAngle testAngle(
    //     -rad(90),
    //     rad(90),
    //     0,
    //     0,
    //     -rad(90),
    //     0);

    /*
    shall be the same pos:
    x:-64.190 y: 311.919 z: 46.724 │ θ1: 125.718 θ2: 24.477 θ3: 11.472 θ4: -58.107 θ5: -94.974 θ6: -2.488 deg | ∞: 1 │ rank: 5 | J11 det: -0.00000000 J22 det: 0.84903973466734450337
    x: 6.043 y: 342.422 z: 61.073 │ θ1: 65.324 θ2: 24.379 θ3: 15.010 θ4: 64.045 θ5: -95.793 θ6: -2.493 deg | ∞: 1 │ rank: 5 | J11 det: -0.00000000 J22 det: 0.89914172499751321421
    */

    JointAngle testAngle = {
        // -rad(90),
        // rad(0),
        // rad(-20),
        // 0,
        // rad(-20),
        // 0);

        // 1.503, -0.375, 1.927, 3.091, 1.604, 759.018); // pointing towards +y
        // 2.415, -0.274, 1.353, 2.287, -1.840, 758.886); // pointing towards -y

        rad(137.563),
        rad(18.423),
        rad(6.517),
        rad(-217.895),
        rad(100.382),
        rad(28451.742)};

    // JointAngle testAngle(
    //     rad(0),
    //     rad(-90),
    //     rad(+90),
    //     rad(0),
    //     rad(+90),
    //     rad(0));

    // tempAngle.theta1 = -testAngle.theta1;
    // tempAngle.theta2 += +rad(90);
    // tempAngle.theta3 = -tempAngle.theta3;
    // tempAngle.theta3 += -rad(90);
    // tempAngle.theta4 += +rad(90);

    // testAngle = tempAngle;
    // motorAngles = testAngle.toMotorPosition();
    cout << mot_r(((M_PI / 2) - tempAngle.theta3)) << endl;
    printf("For θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f deg\r\n",
           deg(testAngle.theta1),
           deg(testAngle.theta2),
           deg(testAngle.theta3),
           deg(testAngle.theta4),
           deg(testAngle.theta5),
           deg(testAngle.theta6));
    auto FK_out = FK(testAngle);
    Coor FK_coor = FK_out.first;

    vector<Matrix4d> frames = FK_out.second;

    printf("FK Coor->     x= %.3f y= %.3f z= %.3f\r\n", FK_coor.x, FK_coor.y, FK_coor.z);
    printf("must be       x: 397.132 y: 17.380 z: 424.683\r\n");
    // printf("mot_pos->     %.3f %.3f %.3f %.3f %.3f %.3f\r\n",
    //        motorAngles.pos1,
    //        motorAngles.pos2,
    //        motorAngles.pos3,
    //        motorAngles.pos4,
    //        motorAngles.pos5,
    //        motorAngles.pos6);
    // cout << "valid  ->     x= 18.403 y= 24.340 z= -608.976" << endl;
    // mayProceed = IK(FK_coor, &tempAngle);
    // printf("valid?->  %d\r\n", mayProceed);
    // for (auto mat : frames)
    //     print_mat(mat, true, "  % 16.10g│");

    return 0;

    for (int i = 0; i < 6; i++)
    {
        Vector3d zi = frames[i].block<3, 1>(0, 2);
        Vector3d Oi = frames[i].block<3, 1>(0, 3);
        for (int j = 0; j < 3; j++)
        {
            if (abs(zi(j)) < thresh)
                zi(j) = 0;
            if (abs(Oi(j)) < thresh)
                Oi(j) = 0;
        }
        cout << "Joint " << i + 1 << " axis: ";
        print_mat(zi.transpose(), false, "% 7.3g");
        cout << ", origin: ";
        print_mat(Oi.transpose(), false, "% 7.3g");
        cout << endl;
    }

    MatrixXd J = createJacobianMatrix(frames);
    auto singular_out = IsSingular(J);
    bool isAtSingularity = singular_out.first;
    cout << "\r\nJacobian matrix:";
    print_mat(J);
    cout << "At ∞: " << (isAtSingularity ? "YES!" : "nah all good") << endl;
    printf("J11 det: % 10.8f J22 det:% .20G\r\n", singular_out.second.first, singular_out.second.second);

    FullPivLU<MatrixXd> luA(J);
    int rank = luA.rank();

    JacobiSVD<MatrixXd> svd;
    svd.compute(J, ComputeThinU | ComputeThinV);
    double min_sv = svd.singularValues().minCoeff();

    double det = J.determinant();

    // // // double det = -100;

    printf("\r\nJ Rank: %d r: %td, SV: %1.10f det: %1.5f\r\n", rank, svd.rank(), min_sv, det);
    cout << "Singulars:";
    print_mat(svd.singularValues().transpose());
    // printf("det:%.5f\r\n", det);

    VectorXd v{{0,   // linear vx
                10,  // linear vy --> move 1 cm/sec upwards
                0,   // linear vz
                0,   // angular vx
                0,   // angular vy
                0}}; // angular vz

    VectorXd velocities = getJointVelocities(J, v);
    velocities /= 2 * M_PI; // devide by 2*M_PI for rev/sec

    print_mat(velocities, true, " % .15f");

    printf("JVel v1:% .10f v2:% .10f v3:% .10f \r\n",
           velocities(0),
           velocities(1),
           velocities(2));

    // FK_out = FK(tempPos.toJointAngle());
    // FK_coor = FK_out.first;
    // printf("FK Motor Coor->     x= %1.2f y= %1.2f z= %1.2f\r\n", FK_coor.x, FK_coor.y, FK_coor.z);
}