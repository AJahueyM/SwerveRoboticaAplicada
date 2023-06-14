// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <fstream>

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandScheduler.h>

#include "SwerveChassis/SwerveChassis.h"
#include "SwerveChassis/PathFollower/SwervePathFollower.h"

struct ResultStep {
  double time;
  double vel_input;
  double vel_target;
};

frc::Translation2d SineYForwardX(units::second_t t){
  units::meter_t x = 1_mps  * t;
  units::meter_t y = 1_mps  * units::second_t(std::sin(t.to<double>()));

  return frc::Translation2d(x, y);
}

frc::Rotation2d SineHeading(units::second_t t){
  return frc::Rotation2d(units::radian_t(M_PI * 0.6 * t.to<double>()));
}

class Robot : public frc::TimedRobot {
 public:
  explicit Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
private:
  SwerveChassis chassis;
  SwervePathFollower follower {chassis, [](units::second_t time) { return SineYForwardX(time);}, [](units::second_t time) { return SineHeading(time);}};

  frc::Joystick joy {0};
  frc2::CommandScheduler& scheduler = frc2::CommandScheduler::GetInstance();

  std::ofstream file;
  bool createdFile = false;
  std::vector<ResultStep> results;
  frc::ChassisSpeeds speed;

  units::second_t time = frc::Timer::GetFPGATimestamp();


};
