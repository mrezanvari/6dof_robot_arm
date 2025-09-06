#ifdef ARDUINO
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#else
#include <Eigen/Jacobi>
#endif

using namespace Eigen;
using Eigen::MatrixXd;

pair<bool, pair<double, double>> IsSingular(MatrixXd &jacobian, double epsilon = 1e-2)
{
  MatrixXd J11(3, 3);
  MatrixXd J22(3, 3);

  J11 = jacobian.block<3, 3>(0, 0);
  J22 = jacobian.block<3, 3>(3, 3);

  // so ik, fk and jacobian use values that are in m but we use mm
  // we know 1mm/1000 = 1m
  // and since J11 is a rotation matrix -> 3 matmuls
  // if we want rescale the final determinant
  // we must do pow(scale of one, 3)
  // so 1000^3 = 1000000000.0;
  double det11 = J11.determinant(); // / 1000000000.0; // removed since we are now using meters
  double det22 = J22.determinant();
  bool singularity = abs(det11 * det22) <= epsilon;
  pair<double, double> determinants = {det11, det22};
  return pair<bool, pair<double, double>>{singularity, determinants};
}

MatrixXd createJacobianMatrix(const vector<Matrix4d> &frames)
{
  /*
  Chapter 4.6.5, Equations 4.56 and 4.58:
  Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). Robot modeling and control (Second edition.). John Wiley & Sons, Inc.
  */
  const int n = frames.size() - 1; // n is the number of joints, but frames includes the Tn at last index

  MatrixXd jacobian(6, n);
  Block o3 = frames.back().block<3, 1>(0, 3);

  for (size_t i = 1; i < frames.size(); ++i)
  {
    size_t ind = i - 1;
    Block zi = frames[i].block<3, 1>(0, 2); // i-1 for regular DH i for MDH
    Block Oi = frames[i].block<3, 1>(0, 3);
    jacobian.block<3, 1>(0, ind) = zi.cross(o3 - Oi);
    jacobian.block<3, 1>(3, ind) = zi;
  }

  return jacobian;
}

VectorXd getJointVelocities(const MatrixXd &J, const VectorXd &v)
{
  // TODO: be sure singularities are handled gracefully!
  auto J_inv = J.inverse();
  return J_inv * v;
}