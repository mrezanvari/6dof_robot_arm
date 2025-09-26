#include <iostream>
#include "../robot_arm_codebase/lib/types.hpp"
#include "../robot_arm_codebase/lib/IKUtils.hpp"
#include "../robot_arm_codebase/lib/FKUtils.hpp"
#include "../robot_arm_codebase/lib/JacobianUtils.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <format>

Coor globalUserPos;
JointAngle tempAngle;

using namespace Eigen;
using Eigen::MatrixXd;

string objToBin(void *obj, size_t obj_size)
{
    unsigned char *pnt = reinterpret_cast<unsigned char *>(obj);
    ostringstream out;
    for (size_t i = 0; i < obj_size; i++)
    {
        for (size_t j = 0; j < 8; j++)
            out << !!((pnt[i] << j) & 0x80);
        out << " ";
    }
    return out.str();
}

template <typename... Args>
string dyna_print(string_view rt_fmt_str, Args &&...args)
{
    return vformat(rt_fmt_str, std::make_format_args(args...));
}
void drawSectionLine(string sectionTitle = "")
{
    const char *sectionLine = "─";
    const size_t sectionLineLength = 161;
    if (sectionTitle != "")
        sectionTitle = "« " + sectionTitle + " »";
    const size_t sectionTitleSize = sectionTitle.size();

    cout << endl;
    for (int i = 0; i < sectionLineLength; ++i)
    {
        if (i == (floor(sectionLineLength / 2) - floor(sectionTitleSize / 2)))
        {
            cout << sectionTitle;
            i += sectionTitleSize;
        }
        else
            cout << sectionLine;
    }
    cout << endl
         << endl;
}

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

template <typename Derived>
void printf_mat(const MatrixBase<Derived> &mat, string &out, bool newline = true, string formatstr = " {: 16.10G}│")
{
    if (newline)
        out += "\r\n";
    for (size_t x = 0; x < mat.rows(); ++x)
    {
        for (size_t y = 0; y < mat.cols(); ++y)
            out += dyna_print(formatstr, mat(x, y));
        if (newline)
            out += "\r\n";
    }
    if (newline)
        out += "\r\n";
}

int main()
{
    drawSectionLine("3 DoF IK"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────
    globalUserPos.x = 280;
    globalUserPos.y = 100;
    globalUserPos.z = -200;

    printf("Coor  ->         x= %1.2f y= %1.2f z= %1.2f\r\n", globalUserPos.x, globalUserPos.y, globalUserPos.z);
    bool mayProceed = IK_Arm(globalUserPos, &tempAngle);

    tempAngle.theta3 = rad(90) - tempAngle.theta3;

    MotorPosition tempPos = tempAngle.toMotorPosition();
    printf("IK Out ->          θ1=%1.3f θ2=%1.3f θ3=%1.3f\r\n\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));
    if (!mayProceed)
        cout << "You shall not pass!" << endl
             << endl;
    else
        cout << "Proceed!" << endl
             << endl;

    drawSectionLine("FK and Jacobian Basics"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    JointAngle testAngle = {
        rad(134.189),
        rad(1.855),
        rad(86.254),
        rad(-1.330),
        rad(78.959),
        rad(95.663)};

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

    // const double thresh = 1e-5;
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

    VectorXd v{{1,   // linear vx
                1,   // linear vy --> move 1 cm/sec upwards
                1,   // linear vz
                1,   // angular vx
                1,   // angular vy
                1}}; // angular vz

    VectorXd velocities(6);
    velocities = extractJointRelation(J, v);
    velocities /= 2 * M_PI; // devide by 2*M_PI for rev/sec
    velocities *= 8;

    print_mat(velocities, true, " % .15f");

    printf("Jacobian Velocites:\r\n");
    for (int i = 0; i < velocities.size(); ++i)
        printf("v%d:% .10f\t", i, velocities(i));

    cout << endl;
    drawSectionLine("6 DoF IK Dev"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    testAngle = {
        rad(134.600),
        rad(43.610),
        rad(18.829),
        rad(0.052),
        rad(61.537),
        rad(0)};

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

    Matrix4d Tn = frames.back();
    Matrix3d Rn = Tn.block<3, 3>(0, 0);

    double theta = atan2(sqrt(1 - pow(Rn(2, 2), 2)), Rn(2, 2));
    double phi = atan2(Rn(1, 2), Rn(0, 2));
    double psi = atan2(Rn(2, 1), -Rn(2, 0));

    // Vector3d O = FK_out.first.toMeter().toVector3d();
    Vector3d O = FK_out.first.toVector3d();
    phi = rad(134.54487);  // 134.54487
    theta = rad(56.02402); // 56.02402
    psi = rad(0);

    printf("\r\nroll/phi: % .5f\r\npitch/theta: % .5f\r\nyaw/psi: % .5f\r\n\r\n", deg(phi), deg(theta), deg(psi));

    Matrix3d R{{-sin(phi) * sin(psi) + cos(phi) * cos(psi) * cos(theta),
                -sin(phi) * cos(psi) - sin(psi) * cos(phi) * cos(theta),
                sin(theta) * cos(phi)},
               {sin(phi) * cos(psi) * cos(theta) + sin(psi) * cos(phi),
                -sin(phi) * sin(psi) * cos(theta) + cos(phi) * cos(psi),
                sin(phi) * sin(theta)},
               {-sin(theta) * cos(psi),
                sin(psi) * sin(theta),
                cos(theta)}};

    cout << "Rotation matrix of R:" << endl;
    print_mat(R);

    Vector3d oc = O - (globalJointParams.back().d * R.col(2));

    Coor ikIn;
    ikIn.x = oc(1);
    ikIn.y = oc(2);
    ikIn.z = oc(0);
    // ikIn.coorScale = Coor::CoorScale::METER;
    tempAngle = JointAngle();
    IK_Arm(ikIn, &tempAngle);
    IKSolution tstSol;
    solve3DoFIK(ikIn, &tstSol);

    FK_out = FK(tempAngle);
    frames = FK_out.second;

    Matrix4d H03 = frames[3];
    Matrix3d R03 = H03.block<3, 3>(0, 0);
    Matrix3d R30 = R03.inverse();
    Matrix3d R36 = R30 * R;

    tempAngle.theta4 = atan2(R36(2, 2), R36(0, 2));
    tempAngle.theta5 = atan2(sqrt(1 - pow(R36(1, 2), 2)), -R36(1, 2));
    tempAngle.theta6 = atan2(-R36(1, 1), R36(1, 0));

    printf("IK Out ->          θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3), deg(tempAngle.theta4), deg(tempAngle.theta5), deg(tempAngle.theta6));
    FK_out = FK(tempAngle);
    printf("FK Coor->      x= %.3f y= %.3f z= %.3f\r\n", FK_out.first.x, FK_out.first.y, FK_out.first.z);

    cout << "────────────────" << endl;
    cout << "New 3DoF IK Solver:" << endl;
    int offsett = 0;
    printf("new IK right up Out ->      ");
    for (int i = offsett; i < offsett + 9; ++i)
        printf("θ%d=% .3f ", i - offsett + 1, deg(tstSol.thetas[i]));
    cout << endl;

    printf("new IK right down Out ->    ");
    offsett = 9;
    for (int i = offsett; i < offsett + 9; ++i)
        printf("θ%d=% .3f ", i - offsett + 1, deg(tstSol.thetas[i]));
    cout << endl
         << endl;

    printf("new IK left up Out ->       ");
    offsett = 18;
    for (int i = offsett; i < offsett + 9; ++i)
        printf("θ%d=% .3f ", i - offsett + 1, deg(tstSol.thetas[i]));
    cout << endl;

    printf("new IK left down Out ->     ");
    offsett = 27;
    for (int i = offsett; i < offsett + 9; ++i)
        printf("θ%d=% .3f ", i - offsett + 1, deg(tstSol.thetas[i]));
    cout << endl
         << endl;

    validate3DoFIKSolutions(&tstSol);
    printf("Solution validation: %s\r\n", objToBin(&tstSol.validationFlags.bits, 1).c_str());

    drawSectionLine("Full IK Test"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    Coor newIKCoor(367.569, 321.100, -362.294);
    Orientation newOrientation(phi, theta, psi);
    JointAngle IKOut;
    bool fullIK_out = IK(newIKCoor, newOrientation, &IKOut);

    printf("IK Out ->          θ1: %1.3f θ2: %1.3f θ3: %1.3f θ4: %1.3f θ5: %1.3f θ6: %1.3f\r\n", deg(IKOut.theta1), deg(IKOut.theta2), deg(IKOut.theta3), deg(IKOut.theta4), deg(IKOut.theta5), deg(IKOut.theta6));
    FK_out = FK(IKOut);
    printf("FK Coor->      x= %.3f y= %.3f z= %.3f\r\n", FK_out.first.x, FK_out.first.y, FK_out.first.z);

    if (!fullIK_out)
        cout << "You shall not pass!" << endl
             << endl;
    else
        cout << "Proceed!" << endl
             << endl;

    drawSectionLine("3 DoF IK with offset following textbook"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────
    globalUserPos.x = 300;
    globalUserPos.y = 320;
    globalUserPos.z = -200;

    // xyz = yzx
    // code - book
    // return pair(Coor(position(1), position(2), position(0)), frames);

    Coor newpos = globalUserPos.toMeter(); // just to keep the names consistent
    tempAngle = JointAngle();

    printf("Coor  ->         x= %1.2f y= %1.2f z= %1.2f\r\n", globalUserPos.x, globalUserPos.y, globalUserPos.z);
    mayProceed = IK_Arm(globalUserPos, &tempAngle);

    tempPos = tempAngle.toMotorPosition();
    printf("IK Out ->          θ1=%1.3f θ2=%1.3f θ3=%1.3f\r\n\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));

    tempAngle.theta1 = tempAngle.theta2 = tempAngle.theta3 = 0;

    double a1 = globalJointParams[0].d; // length of lower arm -> book uses DH, we use MDH so here we use d rather than a
    double a2 = globalJointParams[2].a; // length of lower arm
    double a3 = globalJointParams[3].d; // length of upper arm
    double d = 0;                       // globalJointParams[1].d;  // shoulder offset -> from the second joint DH d parameter

    phi = atan2(newpos.x, newpos.z);
    double r2 = pow(newpos.x, 2) + pow(newpos.z, 2) - pow(d, 2); // r squared -> from equasion 5.24 and 5.26
    double s = (newpos.y - a1);                                  // from equasion 5.24 and 5.26
    double alpha_right = atan2(d, sqrt(r2));
    double alpha_left = atan2(-d, -sqrt(r2));
    double theta1_right = phi - alpha_right; // right arm -> for us
    double theta1_left = phi + alpha_left;   // left arm

    double D = (r2 + pow(s, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3); // from equasion 5.24

    // NOTE: Our quadrants are different from the book
    double theta3_elbow_up = atan2(D, sqrt(1 - pow(D, 2)));
    double theta3_elbow_down = atan2(D, -sqrt(1 - pow(D, 2)));

    double theta2_elbow_up = atan2(s, sqrt(r2)) + atan2(a3 * cos(theta3_elbow_up), a2 + (a3 * sin(theta3_elbow_up)));
    double theta2_elbow_down = atan2(s, sqrt(r2)) + atan2(a3 * cos(theta3_elbow_down), a2 + (a3 * sin(theta3_elbow_down)));

    tempAngle.theta1 = theta1_right;
    tempAngle.theta2 = theta2_elbow_up;
    tempAngle.theta3 = theta3_elbow_up;

    printf("new IK right up Out ->      θ1=% .3f θ2=% .3f θ3=% .3f\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));

    tempAngle.theta1 = theta1_right;
    tempAngle.theta2 = theta2_elbow_down;
    tempAngle.theta3 = theta3_elbow_down;

    printf("new IK right down Out ->    θ1=% .3f θ2=% .3f θ3=% .3f\r\n\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));

    theta2_elbow_up = atan2(s, -sqrt(r2)) + atan2(a3 * cos(theta3_elbow_up), a2 + (a3 * sin(theta3_elbow_up)));
    theta2_elbow_down = atan2(s, -sqrt(r2)) + atan2(a3 * cos(theta3_elbow_down), a2 + (a3 * sin(theta3_elbow_down)));

    tempAngle.theta1 = theta1_left;
    tempAngle.theta2 = theta2_elbow_up;
    tempAngle.theta3 = theta3_elbow_up;

    printf("new IK left up Out ->       θ1=% .3f θ2=% .3f θ3=% .3f\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));

    tempAngle.theta1 = theta1_left;
    tempAngle.theta2 = theta2_elbow_down;
    tempAngle.theta3 = theta3_elbow_down;

    printf("new IK left down Out ->     θ1=% .3f θ2=% .3f θ3=% .3f\r\n", deg(tempAngle.theta1), deg(tempAngle.theta2), deg(tempAngle.theta3));

    cout << "────────────────" << endl;
    IKSolution iksol;
    solve3DoFIK(newpos, &iksol);

    int offset = 0;
    printf("new IK right up Out ->      ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl;

    printf("new IK right down Out ->    ");
    offset = 9;
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl
         << endl;

    printf("new IK left up Out ->       ");
    offset = 18;
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl;

    printf("new IK left down Out ->     ");
    offset = 27;
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl
         << endl;

    validate3DoFIKSolutions(&iksol);
    printf("Solution validation: %s\r\n", objToBin(&iksol.validationFlags.bits, 1).c_str());

    drawSectionLine("6 DoF IK with offset following textbook"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────
    IK(newIKCoor, newOrientation, &IKOut);
    FK_out = FK(IKOut);
    printf("FK Coor->      x=% .3f y=% .3f z=% .3f\r\n", FK_out.first.x, FK_out.first.y, FK_out.first.z);
    printf("IK Out ->                   θ1:% .3f θ2:% .3f θ3:% .3f θ4:% .3f θ5:% .3f θ6:% .3f\r\n\r\n", deg(IKOut.theta1), deg(IKOut.theta2), deg(IKOut.theta3), deg(IKOut.theta4), deg(IKOut.theta5), deg(IKOut.theta6));

    IKOut = JointAngle();
    iksol = solveFullIK(newIKCoor, newOrientation, &IKOut);

    offset = 0;
    printf("new IK right up Out ->      ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl;

    printf("new IK right down Out ->    ");
    offset = 9;
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl
         << endl;

    printf("new IK left up Out ->       ");
    offset = 18;
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl;

    printf("new IK left down Out ->     ");
    offset = 27;
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(iksol.thetas[i]));
    cout << endl
         << endl;

    printf("Solution validation: %s\r\n", objToBin(&iksol.validationFlags.bits, 1).c_str());

    drawSectionLine("Test Coor Z-Up"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    Coor yup_coor(111, 222, 333);
    Coor zup_coor = yup_coor.toZUp();
    Coor newYup = zup_coor.toYUp();

    printf("Y-Up x:% f y:% f z:%  f\r\n", yup_coor.x, yup_coor.y, yup_coor.z);
    printf("Z-Up x:% f y:% f z:%  f\r\n", zup_coor.x, zup_coor.y, zup_coor.z);
    printf("?-Up x:% f y:% f z:%  f\r\n", newYup.x, newYup.y, newYup.z);

    drawSectionLine("Test (-) operator for Coor"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    Coor pos1(100, 200, 300);
    Coor pos2(100, 250, 300);
    Coor err_pos = pos2 - pos1;

    printf("pos1          x:% .3f y:% .3f z:%  .3f\r\n", pos1.x, pos1.y, pos1.z);
    printf("pos2          x:% .3f y:% .3f z:%  .3f\r\n", pos2.x, pos2.y, pos2.z);
    printf("error pos     x:% .3f y:% .3f z:%  .3f\r\n", err_pos.x, err_pos.y, err_pos.z);

    try
    {
        pos2 = pos2.toMeter();
        err_pos = pos2 - pos1;
    }
    catch (exception &ex)
    {
        cout << ex.what() << endl;
    }

    printf("error pos     x:% .3f y:% .3f z:%  .3f\r\n", err_pos.x, err_pos.y, err_pos.z);

    drawSectionLine("Proper velocity calculation flow"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    JointAngle currentAngles = {
        rad(134.600),
        rad(43.610),
        rad(18.829),
        rad(0.052),
        rad(61.537),
        rad(0)};

    auto currentFK_out = FK(currentAngles);
    Coor currentPos = currentFK_out.first;
    Coor desiredPos = currentPos;
    desiredPos.y += 1;

    vector<Matrix4d> currentFrames = currentFK_out.second;
    Tn = currentFrames.back();
    Rn = Tn.block<3, 3>(0, 0);

    theta = atan2(sqrt(1 - sq(Rn(2, 2))), Rn(2, 2));
    phi = atan2(Rn(1, 2), Rn(0, 2));
    psi = atan2(Rn(2, 1), -Rn(2, 0));

    Orientation currentOrientation(phi, theta, psi);
    JointAngle desiredAngles;

    auto ikout = solveFullIK(desiredPos, currentOrientation, &desiredAngles);

    auto desiredFK_out = FK(desiredAngles);
    vector<Matrix4d> desiredFrames = desiredFK_out.second;

    Coor errorPos = desiredPos - currentPos;

    printf("currentPos          x:% .3f y:% .3f z:%  .3f\r\n", currentPos.x, currentPos.y, currentPos.z);
    printf("desiredPos          x:% .3f y:% .3f z:%  .3f\r\n", desiredPos.x, desiredPos.y, desiredPos.z);
    printf("errorPos            x:% .3f y:% .3f z:%  .3f\r\n", errorPos.x, errorPos.y, errorPos.z);

    Matrix3d currentRotation = currentFrames.back().block<3, 3>(0, 0);
    Matrix3d desiredRotaion = desiredFrames.back().block<3, 3>(0, 0);

    Matrix3d errorRotationMatrix = desiredRotaion * currentRotation.inverse();
    AngleAxisd errorRotationAngleAxis(errorRotationMatrix);
    Vector3d errorRotatationVector = errorRotationAngleAxis.angle() * errorRotationAngleAxis.axis();
    MatrixXd newJ = createJacobianMatrix(currentFrames); // jacobian must use the current joint angles

    printf("\r\nroll/phi: % .5f\r\npitch/theta: % .5f\r\nyaw/psi: % .5f\r\n\r\n", deg(phi), deg(theta), deg(psi));

    cout << "\r\nCurrent Rotation Matrix:" << endl;
    print_mat(currentRotation, true, " % .4f|");

    cout << "\r\nDesired Rotation Matrix:" << endl;
    print_mat(desiredRotaion, true, " % .4f|");

    cout << "\r\nError Rotation Matrix:" << endl;
    print_mat(errorRotationMatrix, true, " % .4f|");

    VectorXd e(6);
    e.head(3) = errorPos.toVector3d();
    e.tail(3) = errorRotatationVector;

    double gain = 1;
    MatrixXd K = MatrixXd::Identity(6, 6) * gain;
    VectorXd vee = K * e;

    cout << "Error Vector of linear and angular velocities + gain:\r\n"
         << endl;
    cout << vee << endl;

    VectorXd newVelocities(6);
    newVelocities = extractJointRelation(newJ, vee);
    newVelocities /= 2 * M_PI; // devide by 2*M_PI for rev/sec

    printf("\r\nJacobian Velocites:\r\n");
    for (int i = 0; i < newVelocities.size(); ++i)
        printf("v%d:% .10f\t", i, newVelocities(i));
    cout << endl;
    VectorXd testVels = getJointVelocities(currentAngles, desiredAngles, 10);
    for (int i = 0; i < testVels.size(); ++i)
        printf("v%d:% .10f\t", i, testVels(i));
    cout << endl;

    drawSectionLine("Axis-Angle conversions"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    Matrix3d rotationFromAngleAxis;
    rotationFromAngleAxis = AngleAxisd(phi, Vector3d::UnitZ()) *
                            AngleAxisd(theta, Vector3d::UnitY()) *
                            AngleAxisd(psi, Vector3d::UnitZ());

    Matrix3d RotMat;

    RotMat = Matrix3d{{-sin(phi) * sin(psi) + cos(phi) * cos(psi) * cos(theta),
                       -sin(phi) * cos(psi) - sin(psi) * cos(phi) * cos(theta),
                       sin(theta) * cos(phi)},
                      {sin(phi) * cos(psi) * cos(theta) + sin(psi) * cos(phi),
                       -sin(phi) * sin(psi) * cos(theta) + cos(phi) * cos(psi),
                       sin(phi) * sin(theta)},
                      {-sin(theta) * cos(psi),
                       sin(psi) * sin(theta),
                       cos(theta)}};

    cout << "Roataion Matrix from angle-axis:" << endl;
    print_mat(rotationFromAngleAxis);

    cout << "Roataion Matrix from book ZYZ:" << endl;
    print_mat(RotMat);

    fstream r_logger;
    r_logger.open("sim_rot.log", fstream::out);

    int iter_i = 0;
    const int iter_end = 5000;

    string r_logBuffer;

    const double saveTheta = theta;
    int movesign = 1;
    theta = rad(-180);

    while (++iter_i < iter_end)
    {
        theta += rad((movesign * 1));
        if (theta >= rad(180))
            movesign = -1;
        else if (theta <= rad(-180))
            movesign = 1;

        rotationFromAngleAxis = AngleAxisd(phi, Vector3d::UnitZ()) *
                                AngleAxisd(theta, Vector3d::UnitY()) *
                                AngleAxisd(psi, Vector3d::UnitZ());

        RotMat = Matrix3d{{-sin(phi) * sin(psi) + cos(phi) * cos(psi) * cos(theta),
                           -sin(phi) * cos(psi) - sin(psi) * cos(phi) * cos(theta),
                           sin(theta) * cos(phi)},
                          {sin(phi) * cos(psi) * cos(theta) + sin(psi) * cos(phi),
                           -sin(phi) * sin(psi) * cos(theta) + cos(phi) * cos(psi),
                           sin(phi) * sin(theta)},
                          {-sin(theta) * cos(psi),
                           sin(psi) * sin(theta),
                           cos(theta)}};

        // r_logBuffer = dyna_print("For theta {} Roataion Matrix from angle-axis:", theta);
        // printf_mat(rotationFromAngleAxis, r_logBuffer);

        // r_logBuffer += dyna_print("For theta {} Roataion Matrix from book ZYZ :", theta);
        // printf_mat(RotMat, r_logBuffer);

        MatrixXd delta = rotationFromAngleAxis * RotMat.inverse();
        double threshold = 1e-8;
        delta = (threshold < delta.array().abs()).select(delta, 0.0f);
        r_logBuffer = dyna_print("For theta {} Roataion Matrix delta:", theta);
        printf_mat(delta, r_logBuffer);

        r_logger << r_logBuffer;
    }

    r_logger.close();
    theta = saveTheta;
    drawSectionLine("Pick Best solution test"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    newIKCoor = Coor(
        410,
        215,
        0);
    Orientation newIKOrientation(
        rad(90),
        rad(90),
        0);

    JointAngle thisNewJoint;
    JointAngle lastStableJoint;

    IKSolution newSolution = solveFullIK(newIKCoor, newIKOrientation, &lastStableJoint);
    offset = 0;
    printf("new IK right up Out ->      ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(newSolution.thetas[i]));
    cout << endl;

    // printf("new IK right down Out ->    ");
    // offset = 9;
    // for (int i = offset; i < offset + 9; ++i)
    //     printf("θ%d=% .3f ", i - offset + 1, deg(newSolution.thetas[i]));
    // cout << endl
    //      << endl;

    // printf("new IK left up Out ->       ");
    // offset = 18;
    // for (int i = offset; i < offset + 9; ++i)
    //     printf("θ%d=% .3f ", i - offset + 1, deg(newSolution.thetas[i]));
    // cout << endl;

    // printf("new IK left down Out ->     ");
    // offset = 27;
    // for (int i = offset; i < offset + 9; ++i)
    //     printf("θ%d=% .3f ", i - offset + 1, deg(newSolution.thetas[i]));
    cout << endl
         << endl;
    printf("Solutions: %s\r\n", bitset<8>(newSolution.validationFlags.bits).to_string().c_str());

    newIKOrientation.theta = -60;
    newSolution = solveFullIK(newIKCoor, newIKOrientation, &thisNewJoint);

    offset = 0;
    printf("new IK right up Out ->      ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(newSolution.thetas[i]));
    cout << endl;
    printf("Solutions: %s\r\n", bitset<8>(newSolution.validationFlags.bits).to_string().c_str());

    JointAngle best;
    double chose = pickBestSolution(newSolution, lastStableJoint, &best);
    printf("Chose: %f\r\n", chose);

    printf("Last Stable Solution  ->     ");
    for (int i = 0; i < 6; ++i)
        printf("θ%d=% .3f ", i + 1, deg(lastStableJoint.thetas[i]));
    cout << endl;
    printf("Best Selected Solution ->    ");
    for (int i = 0; i < 6; ++i)
        printf("θ%d=% .3f ", i + 1, deg(best.thetas[i]));
    cout << endl;

    drawSectionLine("Continious Motion Simulation"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    cout << "To begin simulation, type 'yes':" << endl;
    string usrInput;
    cin >> usrInput;
    if (!(usrInput == "yes" || usrInput == "y"))
        return 0;

    printf("\r\n\r\n");

    MotorPosition currentMotorPosition;
    double globalVelocity = 0.7;
    Orientation devOrientation(
        rad(90),
        rad(90),
        0);

    newIKCoor = Coor(
        410,
        215,
        0);

    int moveDir = 1;

    int loopUpperBound = 2000;
    int loopCount = 0;
    fstream p_logger;
    p_logger.open("sim_pos1.log", fstream::out);

    fstream v_logger;
    v_logger.open("sim_vel1.log", fstream::out);
    JointAngle currentJointAngles;

    solveFullIK(newIKCoor, devOrientation, &currentJointAngles);

    string p_logBuffer;
    string v_logBuffer;
    while (++loopCount < loopUpperBound)
    {
        devOrientation.theta += rad((moveDir * 0.1));
        // devOrientation.phi += rad((-moveDir * 0.1));

        // TODO: Make the ball rotate around itself -> this would be a starting point:
        // double thet = constrain(rad(30) + sin(millis() * 0.0001), rad(30), rad(100));
        // double ph = constrain(rad(90) + cos(millis() * 0.0001), rad(90), rad(180));
        // printf("theta:% .5f phi:% .5f t2:% .5f p2:% .5f\r\n", deg(thet), deg(ph), deg(devOrientation.theta), deg(devOrientation.phi));

        if (devOrientation.theta > rad(180))
            moveDir = -1;
        else if (devOrientation.theta <= rad(60))
            moveDir = 1;

        // newIKCoor.z += 1;
        // if (newIKCoor.z > -50)
        //     break;

        JointAngle desiredJointAngles;
        IKSolution fullIKSolution = solveFullIK(newIKCoor, devOrientation, &desiredJointAngles);

        double delta = pickBestSolution(fullIKSolution, currentJointAngles, &desiredJointAngles);

        // if (deg(devOrientation.theta) >= 120)
        //     for (int i = 6; i < 9; ++i)
        //         desiredJointAngles.thetas[i - 3] = fullIKSolution.thetas[i];

        currentMotorPosition = currentJointAngles.toMotorPosition();
        FK_out = FK(currentMotorPosition.toJointAngle());
        FK_coor = FK_out.first;
        J = createJacobianMatrix(FK_out.second);
        auto singular_out = IsSingular(J, 1e-4);
        isAtSingularity = singular_out.first;

        FullPivLU<MatrixXd> fivLU(J);
        int rank = fivLU.rank();

        VectorXd jointVelocities = getJointVelocities(currentJointAngles, desiredJointAngles, 10); // gain really high because the delta is too low
        // jointVelocities = jointVelocities.cwiseMin(2);

        p_logBuffer = dyna_print("x:{: 3.3f} y:{: 3.3f} z:{: 3.3f} │ t0:{: .3f} t1:{: .3f} t2:{: .3f} t3:{: .3f} t4:{: .3f} t5:{: .3f} │ phi:{: .3f} theta:{: .3f} psi:{: .3f} │ {} │ J11 det:{: .5f} J22 det:{: .5f} │ rank:{} │ ∞: {:d} │ {}\r\n",
                                 FK_coor.y,
                                 FK_coor.z,
                                 FK_coor.x,
                                 deg(desiredJointAngles.theta1),
                                 deg(desiredJointAngles.theta2),
                                 deg(desiredJointAngles.theta3),
                                 deg(desiredJointAngles.theta4),
                                 deg(desiredJointAngles.theta5),
                                 deg(desiredJointAngles.theta6),
                                 deg(devOrientation.phi),
                                 deg(devOrientation.theta),
                                 deg(devOrientation.psi),
                                 bitset<8>(fullIKSolution.validationFlags.bits).to_string(),
                                 singular_out.second.first,
                                 singular_out.second.second,
                                 rank,
                                 isAtSingularity,
                                 delta);

        // p_logBuffer = dyna_print("x:{: 3.3f} y:{: 3.3f} z:{: 3.3f} │ t0:{: .3f} t1:{: .3f} t2:{: .3f} t3:{: .3f} t4:{: .3f} t5:{: .3f} │ phi:{: .3f} theta:{: .3f} psi:{: .3f} │ {} │ J11 det:{: .5f} J22 det:{: .5f} │ rank:{} │ ∞: {:d} │ {}\r\n",
        //                          FK_coor.y,
        //                          FK_coor.z,
        //                          FK_coor.x,
        //                          deg(fullIKSolution.right.up.theta1),
        //                          deg(fullIKSolution.right.up.theta2),
        //                          deg(fullIKSolution.right.up.theta3),
        //                          deg(fullIKSolution.right.up.wrist2.theta1),
        //                          deg(fullIKSolution.right.up.wrist2.theta2),
        //                          deg(fullIKSolution.right.up.wrist2.theta3),
        //                          deg(devOrientation.phi),
        //                          deg(devOrientation.theta),
        //                          deg(devOrientation.psi),
        //                          bitset<8>(fullIKSolution.validationFlags.bits).to_string(),
        //                          singular_out.second.first,
        //                          singular_out.second.second,
        //                          rank,
        //                          isAtSingularity,
        //                          delta);

        v_logBuffer = "";

        for (int i = 0; i < jointVelocities.size(); ++i)
            v_logBuffer += dyna_print("v{}:{:.10f}\t", i, jointVelocities(i));

        v_logBuffer += "\r\n";

        if (loopCount < 10)
            cout << p_logBuffer;

        p_logger << p_logBuffer;
        v_logger << v_logBuffer;

        currentJointAngles = desiredJointAngles;

        // if (deg(currentJointAngles.theta5) <= 1)
        //     break;
    }
    cout << "Output truncated... -> See \"sim_log.log\"" << endl;
    p_logger.close();
    v_logger.close();

    drawSectionLine("Continious Motion Simulation 2"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    cout << "To begin second simulation, type 'yes':" << endl;
    usrInput = "";
    cin >> usrInput;
    if (!(usrInput == "yes" || usrInput == "y"))
        return 0;

    printf("\r\n\r\n");

    currentMotorPosition = MotorPosition();
    globalVelocity = 0.7;
    Orientation startOrientation(
        rad(90),
        rad(90),
        0);

    Coor startPosition(
        410,
        215,
        0);

    Orientation targetOrientation(
        rad(90),
        rad(90),
        0);

    Coor targetPosition(
        410,
        215,
        -360);

    fstream p_logger_line;
    p_logger_line.open("sim_pos2.log", fstream::out);

    fstream v_logger_line;
    v_logger_line.open("sim_vel2.log", fstream::out);

    string p_logBuffer_line;
    string v_logBuffer_line;

    JointAngle targetJointAngles;
    JointAngle startJointAngles;

    IKSolution currentPosIK = solveFullIK(startPosition, startOrientation, &startJointAngles);
    currentMotorPosition = startJointAngles.toMotorPosition();

    IKSolution targetIK = solveFullIK(targetPosition, targetOrientation, &targetJointAngles);
    int outBest = pickBestSolution(targetIK, startJointAngles, &targetJointAngles);

    offset = 0;
    printf("current IK right up Out ->      ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(currentPosIK.thetas[i]));
    cout << endl;
    printf("new IK right up Out ->          ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(targetIK.thetas[i]));
    cout << endl;

    printf("Chose solution: %d\r\n", outBest);

    bool isAtTarget = false;
    loopCount = 0;
    loopUpperBound = 6000;

    while ((++loopCount < loopUpperBound) && !isAtTarget)
    {
        currentJointAngles = currentMotorPosition.toJointAngle();
        FK_out = FK(currentJointAngles);
        FK_coor = FK_out.first;
        J = createJacobianMatrix(FK_out.second);
        VectorXd jointVelocities = getJointVelocities(currentJointAngles, targetJointAngles, 0.025); // gain really high because the delta is too low

        p_logBuffer_line = dyna_print("x:{: 3.3f} y:{: 3.3f} z:{: 3.3f} │ t0:{: .3f} t1:{: .3f} t2:{: .3f} t3:{: .3f} t4:{: .3f} t5:{: .3f} │ phi:{: .3f} theta:{: .3f} psi:{: .3f} │ {} \r\n",
                                      FK_coor.y,
                                      FK_coor.z,
                                      FK_coor.x,
                                      deg(currentJointAngles.theta1),
                                      deg(currentJointAngles.theta2),
                                      deg(currentJointAngles.theta3),
                                      deg(currentJointAngles.theta4),
                                      deg(currentJointAngles.theta5),
                                      deg(currentJointAngles.theta6),
                                      deg(targetOrientation.phi),
                                      deg(targetOrientation.theta),
                                      deg(targetOrientation.psi),
                                      bitset<8>(targetIK.validationFlags.bits).to_string());

        v_logBuffer_line = "";

        for (int i = 0; i < jointVelocities.size(); ++i)
            v_logBuffer_line += dyna_print("v{}:{:.10f}\t", i, jointVelocities(i));

        v_logBuffer_line += "\r\n";

        if (loopCount < 10)
            cout << p_logBuffer_line;

        p_logger_line << p_logBuffer_line;
        v_logger_line << v_logBuffer_line;

        currentMotorPosition.pos1 += jointVelocities(0);
        currentMotorPosition.pos2 += jointVelocities(1);
        currentMotorPosition.pos3 += jointVelocities(2);
        currentMotorPosition.pos4 += jointVelocities(3);
        currentMotorPosition.pos5 += jointVelocities(4);
        currentMotorPosition.pos6 += jointVelocities(5);

        if (FK_coor.toYUp().isEqualOrClose(targetPosition))
            isAtTarget = true;
    }
    cout << "Output truncated... -> See \"sim_log.log\"" << endl;
    p_logger_line.close();
    v_logger_line.close();

    drawSectionLine("Continious Motion Simulation 3"); // ──────────────────────────────────────────────────────────────────────────────────────────────────────────

    cout << "To begin second simulation, type 'yes':" << endl;
    usrInput = "";
    cin >> usrInput;
    if (!(usrInput == "yes" || usrInput == "y"))
        return 0;

    printf("\r\n\r\n");

    currentMotorPosition = MotorPosition();
    globalVelocity = 0.7;
    startOrientation = Orientation(
        rad(90),
        rad(90),
        0);

    startPosition = Coor(
        410,
        215,
        0);

    targetOrientation = Orientation(
        rad(90),
        rad(90),
        0);

    targetPosition = Coor(
        410,
        215,
        0);

    p_logger_line.open("sim_pos3.log", fstream::out);
    v_logger_line.open("sim_vel3.log", fstream::out);

    currentPosIK = solveFullIK(startPosition, startOrientation, &startJointAngles);
    currentMotorPosition = startJointAngles.toMotorPosition();

    targetIK = solveFullIK(targetPosition, targetOrientation, &targetJointAngles);

    targetJointAngles = startJointAngles;

    offset = 0;
    printf("current IK right up Out ->      ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(currentPosIK.thetas[i]));
    cout << endl;
    printf("new IK right up Out ->          ");
    for (int i = offset; i < offset + 9; ++i)
        printf("θ%d=% .3f ", i - offset + 1, deg(targetIK.thetas[i]));
    cout << endl;

    printf("Chose solution: %d\r\n", outBest);

    loopCount = 0;
    loopUpperBound = 6000;

    moveDir = 1;

    while (++loopCount < loopUpperBound)
    {
        startOrientation.theta += rad((moveDir * 0.1));
        if (startOrientation.theta > rad(180))
            moveDir = -1;
        else if (startOrientation.theta <= rad(60))
            moveDir = 1;

        solveFullIK(targetPosition, startOrientation, &targetJointAngles);

        currentJointAngles = currentMotorPosition.toJointAngle();
        FK_out = FK(currentJointAngles);
        FK_coor = FK_out.first;
        J = createJacobianMatrix(FK_out.second);
        VectorXd jointVelocities = getJointVelocities(currentJointAngles, targetJointAngles, 0.2); // gain really high because the delta is too low

        p_logBuffer_line = dyna_print("x:{: 3.3f} y:{: 3.3f} z:{: 3.3f} │ t0:{: .3f} t1:{: .3f} t2:{: .3f} t3:{: .3f} t4:{: .3f} t5:{: .3f} │ phi:{: .3f} theta:{: .3f} psi:{: .3f} │ {} \r\n",
                                      FK_coor.y,
                                      FK_coor.z,
                                      FK_coor.x,
                                      deg(currentJointAngles.theta1),
                                      deg(currentJointAngles.theta2),
                                      deg(currentJointAngles.theta3),
                                      deg(currentJointAngles.theta4),
                                      deg(currentJointAngles.theta5),
                                      deg(currentJointAngles.theta6),
                                      deg(startOrientation.phi),
                                      deg(startOrientation.theta),
                                      deg(startOrientation.psi),
                                      bitset<8>(targetIK.validationFlags.bits).to_string());

        v_logBuffer_line = "";

        for (int i = 0; i < jointVelocities.size(); ++i)
            v_logBuffer_line += dyna_print("v{}:{:.10f}\t", i, jointVelocities(i));

        v_logBuffer_line += "\r\n";

        if (loopCount < 10)
            cout << p_logBuffer_line;

        p_logger_line << p_logBuffer_line;
        v_logger_line << v_logBuffer_line;

        currentMotorPosition.pos1 += jointVelocities(0);
        currentMotorPosition.pos2 += jointVelocities(1);
        currentMotorPosition.pos3 += jointVelocities(2);
        currentMotorPosition.pos4 += jointVelocities(3);
        currentMotorPosition.pos5 += jointVelocities(4);
        currentMotorPosition.pos6 += jointVelocities(5);
    }
    cout << "Output truncated... -> See \"sim_log.log\"" << endl;
    p_logger_line.close();
    v_logger_line.close();
}