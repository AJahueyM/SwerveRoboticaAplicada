// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "Utils.h"

Robot::Robot() : TimedRobot(10_ms) {}

void Robot::RobotInit() { scheduler.Enable(); }

void Robot::RobotPeriodic() { scheduler.Run(); }

void Robot::AutonomousInit() {
	auto pose = chassis.GetEstimatedPose();
	follower.Reset(pose.X(), pose.Y(), pose.Rotation().Radians());
}

void Robot::AutonomousPeriodic() {
	follower.Update();
}

void Robot::TeleopInit() {
	// time = frc::Timer::GetFPGATimestamp();
	// file.open("/home/lvuser/out2.csv");
}

void Robot::TeleopPeriodic() {
	double angularSpeed = Utils::ApplyAxisFilter(-joy.GetRawAxis(4)) * 2 * M_PI;
	double linearSpeedX = Utils::ApplyAxisFilter(-joy.GetRawAxis(1)) * 2;
	double linearSpeedY = Utils::ApplyAxisFilter(-joy.GetRawAxis(0)) * 2;

	frc::Translation2d targetVels {units::meter_t(linearSpeedX), units::meter_t(linearSpeedY)};
	targetVels = targetVels.RotateBy(-chassis.GetEstimatedPose().Rotation());

	frc::ChassisSpeeds speeds;
	speeds.omega = units::radians_per_second_t(angularSpeed);
	speeds.vx = units::meters_per_second_t(targetVels.X().to<double>());
	speeds.vy = units::meters_per_second_t(targetVels.Y().to<double>());

	chassis.SetChassisSpeeds(speeds);

	if(joy.GetRawButton(7)){
		chassis.ResetHeading();
	}


	// units::second_t elapsed = frc::Timer::GetFPGATimestamp() - time;


	// if(elapsed < 10_s){
	// 	ResultStep step;


	// 	step.time = elapsed.to<double>();
	// 	step.vel_input = speed.omega.to<double>();
	// 	step.vel_target = chassis.GetChassisSpeeds().omega.to<double>();

	// 	results.emplace_back(step);

	// 	if(elapsed > 2_s){
	// 		speed.omega = 1_rad_per_s;
	// 	}
		
	// 	if(elapsed > 4_s){
	// 		speed.omega = 2_rad_per_s;
	// 	}
		
	// 	if(elapsed > 6_s){
	// 		speed.omega = 3_rad_per_s;
	// 	}


	// }else if(!createdFile){
	// 	speed.omega = 0_rad_per_s;
	// 	createdFile = true;

	// 	for(auto& step : results){
	// 		file << step.time << ',' << step.vel_input << ',' << step.vel_target << std::endl;
	// 	}

	// }

	// chassis.SetChassisSpeeds(speed);


}

void Robot::DisabledInit() {
	// file.close();

}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
