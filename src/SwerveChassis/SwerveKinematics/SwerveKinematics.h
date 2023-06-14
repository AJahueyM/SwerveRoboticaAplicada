// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <wpi/array.h>
#include <units/acceleration.h>
#include "Eigen/QR"

struct SwerveKinematicsParams { //Translation of each module to the center of mass of the chassis
  frc::Translation2d topRightTranslation;
  frc::Translation2d topLeftTranslation;
  frc::Translation2d bottomRightTranslation;
  frc::Translation2d bottomLeftTranslation;
};

struct InverseKinematicCommand {
  units::radian_t heading = 0_rad;
  units::radians_per_second_t headingVelocity = 0_rad_per_s;
  units::meters_per_second_t wheelVelocity = 0_mps;
  units::meters_per_second_squared_t wheelAcceleration = 0_mps_sq;
};

class SwerveKinematics {
public:
  SwerveKinematics(const wpi::array<frc::Translation2d, 4>& modulePos);
  wpi::array<frc::SwerveModuleState, 4> ToSwerveModuleStates(const frc::ChassisSpeeds& chassisSpeeds, const wpi::array<frc::SwerveModuleState, 4>& currentStates,  units::second_t timestep = 0.01_s);

private:
  InverseKinematicCommand GetModuleCommand(const double v_mx, const double v_my, const double a_mx, const double a_my);

  Eigen::MatrixXf A {8, 3};
  Eigen::MatrixXf chassisSpeedsM {3, 1};
  
  Eigen::MatrixXf moduleSpeeds {8, 1};
  Eigen::MatrixXf moduleSpeedsPrev {8, 1};

  Eigen::MatrixXf moduleAccels {8, 1};
  wpi::array<frc::SwerveModuleState, 4> targetStates {frc::SwerveModuleState(), frc::SwerveModuleState(), frc::SwerveModuleState(), frc::SwerveModuleState()};

  SwerveKinematicsParams params;

  units::radian_t testHeading = 0_rad;
};
