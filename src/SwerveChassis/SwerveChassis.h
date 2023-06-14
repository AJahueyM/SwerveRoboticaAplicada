// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <AHRS.h>

#include "SwerveModule/SwerveModule.h"
#include "SwerveKinematics/SwerveKinematics.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>


class SwerveChassis : public frc2::SubsystemBase {
public:
	SwerveChassis();

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	void Periodic() override;

  void SetChassisSpeeds(const frc::ChassisSpeeds& speeds);
  void ResetHeading();

  const wpi::array<frc::SwerveModuleState, 4>&  GetModuleStates();
  const frc::Pose2d&  GetEstimatedPose();
  frc::ChassisSpeeds GetChassisSpeeds();

private:
  void PublishTelemetry();
  std::vector<SwerveModule*> modules;
  wpi::array<frc::SwerveModuleState, 4> moduleWPIStates {
    topRight.GetWPIState(),
    topLeft.GetWPIState(),
    bottomRight.GetWPIState(),
    bottomLeft.GetWPIState()
  };

  SwerveModule topRight {2, 1, 11, {-2.808713_rad}};
  SwerveModule topLeft {4, 3, 12, {-2.702869_rad}};
  SwerveModule bottomRight {6, 5, 13, {0.516950_rad}};
  SwerveModule bottomLeft {8, 7, 14, {2.356190_rad}};

  std::array<frc::Translation2d, 4> moduleTranslations{
      frc::Translation2d(10.36_in, -10.36_in),  // front right
      frc::Translation2d(10.36_in, 10.36_in),   // front left
      frc::Translation2d(-10.36_in, -10.36_in), // back right
      frc::Translation2d(-10.36_in, 10.36_in)   // back left
  };

  std::array<frc::SwerveModulePosition, 4> modulesPos{
      frc::SwerveModulePosition(),
      frc::SwerveModulePosition(),
      frc::SwerveModulePosition(),
      frc::SwerveModulePosition()
  };

  SwerveKinematics kinematics {moduleTranslations};

  frc::SwerveDriveKinematics<4> wpiKinematics{ moduleTranslations };


  frc::SwerveDrivePoseEstimator<4> odometry{
    wpiKinematics,
    frc::Rotation2d{},
    modulesPos,
    frc::Pose2d{}
  };

  frc::Pose2d estimatedPose;

  frc::ChassisSpeeds targetChassisSpeeds;
  units::second_t lastUpdateTime = frc::Timer::GetFPGATimestamp();
  AHRS navx{ frc::SPI::Port::kMXP };


};
