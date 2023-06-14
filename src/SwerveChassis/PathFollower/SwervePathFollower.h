// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/controller/ProfiledPIDController.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include "SwerveChassis/SwerveChassis.h"


class SwervePathFollower {
 public:
  SwervePathFollower(SwerveChassis& chassis, std::function<frc::Translation2d(units::second_t)> translationTarget, std::function<frc::Rotation2d(units::second_t)> headingTarget);
  void Update();
  void Reset(units::meter_t x, units::meter_t y, units::radian_t omega);
private:
  frc::ProfiledPIDController<units::meter> xPID { 3.6029, 0.0, 0.000294, {2_mps, 3_mps_sq}, 10_ms};
  frc::ProfiledPIDController<units::meter> yPID { 3.6029, 0.0, 0.000294, {2_mps, 3_mps_sq}, 10_ms};
  frc::ProfiledPIDController<units::radian> headingPID {9.0090, 0.0, 0.008027, {6_rad_per_s, 3_rad_per_s_sq}, 10_ms};

  SwerveChassis& chassis;
  units::second_t startTime;
  std::function<frc::Translation2d(units::second_t)> translationTarget;
  std::function<frc::Rotation2d(units::second_t)> headingTarget;
};
