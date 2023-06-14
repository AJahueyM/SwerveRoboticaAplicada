// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwervePathFollower.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwervePathFollower::SwervePathFollower(SwerveChassis& chassis, std::function<frc::Translation2d(units::second_t)> translationTarget, std::function<frc::Rotation2d(units::second_t)> headingTarget) : chassis(chassis), translationTarget(translationTarget), headingTarget(headingTarget){
    xPID.SetTolerance(0.01_m);
    yPID.SetTolerance(0.01_m);
    headingPID.SetTolerance(0.005_rad);

    headingPID.EnableContinuousInput(units::radian_t(-M_PI), units::radian_t(M_PI));
}

void SwervePathFollower::Update(){
    units::second_t elapsed = frc::Timer::GetFPGATimestamp() - startTime;
    frc::Translation2d pointTarget = translationTarget(elapsed);
    frc::Rotation2d heading = headingTarget(elapsed);

    frc::Pose2d currentPose = chassis.GetEstimatedPose();

    double vx = xPID.AtGoal() ? 0.0 : xPID.Calculate(currentPose.X(), pointTarget.X());
    double vy = yPID.AtGoal() ? 0.0 : yPID.Calculate(currentPose.Y(), pointTarget.Y());
    double omega = headingPID.AtGoal() ? 0.0 : headingPID.Calculate(currentPose.Rotation().Radians(), heading.Radians());

    frc::Translation2d commandVels {units::meter_t(vx), units::meter_t(vy)};
    commandVels = commandVels.RotateBy(-currentPose.Rotation());

    frc::ChassisSpeeds commandedSpeeds;
    commandedSpeeds.vx = commandVels.X() / 1_s;
    commandedSpeeds.vy = commandVels.Y() / 1_s;
    commandedSpeeds.omega = units::radians_per_second_t(omega);

    frc::SmartDashboard::PutNumber("SwervePathFollower/Vx", commandedSpeeds.vx.to<double>());
    frc::SmartDashboard::PutNumber("SwervePathFollower/Vy", commandedSpeeds.vy.to<double>());
    frc::SmartDashboard::PutNumber("SwervePathFollower/Omega", commandedSpeeds.omega.to<double>());

    chassis.SetChassisSpeeds(commandedSpeeds);
}

void SwervePathFollower::Reset(units::meter_t x, units::meter_t y, units::radian_t omega){
    xPID.Reset(x);
    yPID.Reset(y);
    headingPID.Reset(omega);
    startTime = frc::Timer::GetFPGATimestamp();
}
