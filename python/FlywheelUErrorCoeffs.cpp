#include "subsystems/\FlywheelUErrorCoeffs.hpp"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<2, 1, 1> MakeFlywheelUErrorPlantCoeffs() {
  Eigen::Matrix<double, 2, 2> A;
  A(0, 0) = 0.970595658870265;
  A(0, 1) = 0.8344721004414811;
  A(1, 0) = 0.0;
  A(1, 1) = 0.0;
  Eigen::Matrix<double, 2, 1> B;
  B(0, 0) = 0.8344721004414811;
  B(1, 0) = 0.0;
  Eigen::Matrix<double, 1, 2> C;
  C(0, 0) = 1;
  C(0, 1) = 0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;
  return frc::StateSpacePlantCoeffs<2, 1, 1>(A, B, C, D);
}

frc::StateSpaceControllerCoeffs<2, 1, 1> MakeFlywheelUErrorControllerCoeffs() {
  Eigen::Matrix<double, 1, 2> K;
  K(0, 0) = 0.7365346559390376;
  K(0, 1) = 1.0;
  Eigen::Matrix<double, 1, 2> Kff;
  Kff(0, 0) = 0.635754996350343;
  Kff(0, 1) = 0.0;
  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;
  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<2, 1, 1>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<2, 1, 1> MakeFlywheelUErrorObserverCoeffs() {
  Eigen::Matrix<double, 2, 1> K;
  K(0, 0) = 0.9999000194158469;
  K(1, 0) = 0.0;
  return frc::StateSpaceObserverCoeffs<2, 1, 1>(K);
}

frc::StateSpaceLoop<2, 1, 1> MakeFlywheelUErrorLoop() {
  return frc::StateSpaceLoop<2, 1, 1>(MakeFlywheelUErrorPlantCoeffs(),
                                      MakeFlywheelUErrorControllerCoeffs(),
                                      MakeFlywheelUErrorObserverCoeffs());
}
