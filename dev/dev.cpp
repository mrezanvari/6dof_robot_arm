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
    globalUserPos.x = 280;
    globalUserPos.y = 100;
    globalUserPos.z = -200;

    printf("Coor  ->         x= %1.2f y= %1.2f z= %1.2f\r\n", globalUserPos.x, globalUserPos.y, globalUserPos.z);
    bool mayProceed = IK(globalUserPos, &tempAngle);

    tempAngle.theta3 = rad(90) - tempAngle.theta3;

    MotorPosition tempPos = tempAngle.toMotorPosition();
    printf("IK Out ->          θ1=%1.3f θ2=%1.3f θ3=%1.3f\r\n\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));
    if (!mayProceed)
        cout << "You shall not pass!" << endl
             << endl;
    else
        cout << "Proceed!" << endl
             << endl;

    cout << "──────────────────────────────────────────────────────────────────────────────────────────────────────────" << endl;

    JointAngle testAngle = {
        rad(0),
        rad(60),
        rad(0),
        rad(-70),
        rad(-120),
        rad(0)};

    // JointAngle testAngle(
    // rad(0),
    // rad(-90),
    // rad(+90),
    // rad(0),
    // rad(+90),
    // rad(0));

    printf("\r\nFor θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f deg\r\n",
           deg(testAngle.theta1),
           deg(testAngle.theta2),
           deg(testAngle.theta3),
           deg(testAngle.theta4),

           deg(testAngle.theta5),
           deg(testAngle.theta6));
    auto FK_out = FK(testAngle);
    Coor FK_coor = FK_out.first;

    vector<Matrix4d> frames = FK_out.second;

    printf("FK Coor->      x= %.3f y= %.3f z= %.3f\r\n", FK_coor.x, FK_coor.y, FK_coor.z);
    FK_coor = FK_precise(testAngle);
    printf("Fast FK Coor-> x= %.3f y= %.3f z= %.3f\r\n", FK_coor.x, FK_coor.y, FK_coor.z);

    // for (int i = 0; i < 6; i++)
    // {
    //     Vector3d zi = frames[i].block<3, 1>(0, 2);
    //     Vector3d Oi = frames[i].block<3, 1>(0, 3);
    //     for (int j = 0; j < 3; j++)
    //     {
    //         if (abs(zi(j)) < thresh)
    //             zi(j) = 0;
    //         if (abs(Oi(j)) < thresh)
    //             Oi(j) = 0;
    //     }
    //     cout << "Joint " << i + 1 << " axis: ";
    //     print_mat(zi.transpose(), false, "% 7.3g");
    //     cout << ", origin: ";
    //     print_mat(Oi.transpose(), false, "% 7.3g");
    //     cout << endl;
    // }

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

    printf("\r\nJ Rank: %d r: %td, SV: %1.10f det: %1.5f\r\n", rank, svd.rank(), min_sv, det);
    cout << "Singulars:";
    print_mat(svd.singularValues().transpose());

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

    cout << "──────────────────────────────────────────────────────────────────────────────────────────────────────────" << endl;
    // x:-60.632 y: 298.841 z: 76.994 │ θ1: 125.613 θ2: 28.179 θ3: 3.495 θ4: 295.900 θ5: -99.404 θ6: 5454.419 de
    // x:-54.067 y: 327.222 z: 60.034 │ θ1: 117.850 θ2: 30.726 θ3: -29.172 θ4: 48.462 θ5: 87.102 θ6: 35527.159 deg
    // x:-352.765 y: 357.883 z: 311.934 │ θ1: 134.600 θ2: 43.610 θ3: 18.829 θ4: 0.052 θ5: 61.537 θ6: 62546.078 deg | ∞: 0 │ rank: 6 | J11 det:  30450268.80186589 J22 det: 0.87912648256265846225
    testAngle = {
        rad(134.600),
        rad(43.610),
        rad(18.829),
        rad(0.052),
        rad(61.537),
        rad(90)};

    printf("\r\nFor                θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f deg\r\n",
           deg(testAngle.theta1),
           deg(testAngle.theta2),
           deg(testAngle.theta3),
           deg(testAngle.theta4),
           deg(testAngle.theta5),
           deg(testAngle.theta6));

    FK_out = FK(testAngle);
    frames = FK_out.second;

    printf("FK Coor->      x= %.3f y= %.3f z= %.3f\r\n", FK_out.first.x, FK_out.first.y, FK_out.first.z);

    auto Tn = frames.back();
    auto Rn = Tn.block<3, 3>(0, 0);

    print_mat(Tn);

    double theta = acos(Rn(2, 2));
    double phi = atan2(Rn(1, 2), Rn(0, 2));
    double psi = atan2(Rn(2, 1), -Rn(2, 0));

    auto O = Vector3d{{
        FK_out.first.x,
        FK_out.first.y,
        FK_out.first.z,
    }};

    phi = rad(100);
    theta = rad(56.02402);
    psi = rad(0);

    Matrix3d R{{-sin(phi) * sin(psi) + cos(phi) * cos(psi) * cos(theta),
                -sin(phi) * cos(psi) - sin(psi) * cos(phi) * cos(theta),
                sin(theta) * cos(phi)},
               {sin(phi) * cos(psi) * cos(theta) + sin(psi) * cos(phi),
                -sin(phi) * sin(psi) * cos(theta) + cos(phi) * cos(psi),
                sin(phi) * sin(theta)},
               {-sin(theta) * cos(psi),
                sin(psi) * sin(theta),
                cos(theta)}};

    auto oc = O - (globalJointParams.back().d * R.col(2));

    Coor ikIn;
    ikIn.x = oc(1);
    ikIn.y = oc(2);
    ikIn.z = oc(0);
    tempAngle = JointAngle();
    IK(ikIn, &tempAngle);
    FK_out = FK(tempAngle);
    frames = FK_out.second;

    auto H03 = frames[3];
    auto R03 = H03.block<3, 3>(0, 0);
    auto R30 = R03.inverse();
    auto R36 = R30 * R;

    auto ct4 = -R36(1, 2);
    double t5 = acos(ct4);

    tempAngle.theta5 = t5;
    auto ct5 = R36(1, 0) / sin(t5);
    tempAngle.theta6 = acos(ct5);

    auto ct3 = R36(0, 2) / sin(t5);
    tempAngle.theta4 = acos(ct3);

    printf("IK Out ->          θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3), deg(tempAngle.theta4), deg(tempAngle.theta5), deg(tempAngle.theta6));
    FK_out = FK(tempAngle);
    printf("FK Coor->      x= %.3f y= %.3f z= %.3f\r\n", FK_out.first.x, FK_out.first.y, FK_out.first.z);
}