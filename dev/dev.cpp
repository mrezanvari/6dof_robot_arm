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
    FK_coor = fast_FK(testAngle);
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
}