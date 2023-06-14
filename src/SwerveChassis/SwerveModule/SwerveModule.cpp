// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "SwerveModule.h"

#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Utils.h"


SwerveModule::SwerveModule(int steeringId, int wheelID, int canCoderID,
			   frc::Rotation2d headingOffset)
    : steeringMotor(steeringId),
      wheelMotor(wheelID),
      steeringMagnetometer(canCoderID) {
	steeringMotor.ConfigFactoryDefault();
	wheelMotor.ConfigFactoryDefault();
	steeringMagnetometer.ConfigFactoryDefault();

	CANCoderConfiguration cancoderConfig;
	cancoderConfig.absoluteSensorRange = AbsoluteSensorRange::Signed_PlusMinus180;
	cancoderConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
	cancoderConfig.magnetOffsetDegrees = headingOffset.Degrees().to<double>();
    cancoderConfig.sensorCoefficient = 2 * M_PI / 4096.0;
    cancoderConfig.unitString = "rad";
    cancoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;

	int error = steeringMagnetometer.ConfigAllSettings(cancoderConfig, 200);
	if (error != ErrorCode::OK) {
		std::cerr << "Failed to configure CANCoder " << canCoderID
			  << " ErrorCode: " << error << std::endl;
		throw "Could not configure CANCoder";
	}

	TalonFXConfiguration steeringConfig;
	steeringConfig.supplyCurrLimit = SupplyCurrentLimitConfiguration(true, 30, 20, 0.5);
	steeringConfig.voltageCompSaturation = 10;
	steeringConfig.absoluteSensorRange = AbsoluteSensorRange::Signed_PlusMinus180;
	steeringConfig.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
	steeringConfig.neutralDeadband = 0.001;
	steeringConfig.closedloopRamp = 0.1;

	error |= steeringMotor.ConfigAllSettings(steeringConfig, 200);
	error |= steeringMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 200);
	steeringMotor.SetInverted(true);


	steeringMotor.Config_kP(0, 0.080589958292209);
	steeringMotor.Config_kD(0, 4.397616133878558);

	if (error != ErrorCode::OK) {
		std::cerr << "Failed to configure SteeringMotor " << steeringId
			  << " ErrorCode: " << error << std::endl;
		throw "Could not configure SteeringMotor";
	}

	TalonFXConfiguration wheelConfig;
	wheelConfig.supplyCurrLimit = SupplyCurrentLimitConfiguration(true, 30, 20, 0.5);
	wheelConfig.voltageCompSaturation = 10;
	wheelConfig.neutralDeadband = 0.001;
	wheelConfig.closedloopRamp = 0.1;

	error |= wheelMotor.ConfigAllSettings(wheelConfig, 200);
	error |= wheelMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 200);


	wheelMotor.Config_kP(0, 0.018184014117063);
	wheelMotor.Config_kI(0, 3.890784595627245e-04);
	wheelMotor.SetIntegralAccumulator(0);

	if (error != ErrorCode::OK) {
		std::cerr << "Failed to configure WheelMotor " << wheelID
			  << " ErrorCode: " << error << std::endl;
		throw "Could not configure WheelMotor";
	}

	frc::Wait(400_ms);
	steeringMotor.SetSelectedSensorPosition(GetSteeringMotorPosFromHeading(steeringMagnetometer.GetAbsolutePosition()));
	wheelMotor.SetSelectedSensorPosition(0);

		
    wheelMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated, 5);
    wheelMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5);
	
	steeringMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_21_FeedbackIntegrated, 5);
    steeringMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 5);
}

const ModuleState& SwerveModule::GetState() {
	return state;
}

const frc::SwerveModuleState& SwerveModule::GetWPIState() {
	return wpiState;
}

ModuleState SwerveModule::GetState_Int() {
	ModuleState state;
	double steeringMotorPos = steeringMotor.GetSelectedSensorPosition();
	double steeringMotorVel = steeringMotor.GetSelectedSensorVelocity();

	double wheelMotorPos = wheelMotor.GetSelectedSensorPosition();
	double wheelMotorVel = wheelMotor.GetSelectedSensorVelocity();

	state.steeringMotorRawPos = steeringMotorPos;
	state.steeringMotorRawVel = steeringMotorVel;

	state.wheelMotorRawPos = wheelMotorPos;
	state.wheelMotorRawVel = wheelMotorVel;

	state.heading = frc::Rotation2d(units::radian_t(Utils::InputModulus(GetHeadingFromSteeringMotorPos(steeringMotorPos), -M_PI, M_PI)));
	state.headingVelocity = units::radians_per_second_t( steeringMotorVel * RADS_PER_MOTOR_PULSE * 10);

//	state.steeringVoltage = units::volt_t(steeringMotor.GetMotorOutputVoltage());
//	state.wheelVoltage = units::volt_t(wheelMotor.GetMotorOutputVoltage());

    state.distance = units::meter_t(GetDistanceFromWheelMotorPos(wheelMotorPos));
    state.velocity = units::meters_per_second_t(GetMPSFromWheelMotorVel(wheelMotorVel));

	state.steeringClosedLoopError = steeringMotor.GetClosedLoopError();
	state.wheelClosedLoopError = wheelMotor.GetClosedLoopError();

	return state;
}


void SwerveModule::PublishTelemetry(std::string name) {
	frc::SmartDashboard::PutNumber(name + "/Heading",
				       state.heading.Radians().to<double>());
    frc::SmartDashboard::PutNumber(name + "/HeadingDegrees",
				       state.heading.Degrees().to<double>());
    frc::SmartDashboard::PutNumber(name + "/Magnetometer",
				       steeringMagnetometer.GetAbsolutePosition());
	frc::SmartDashboard::PutNumber(name + "/Distance",
				       state.distance.to<double>());
    frc::SmartDashboard::PutNumber(name + "/Velocity",
				       state.velocity.to<double>());
	frc::SmartDashboard::PutNumber(name + "/SteeringClosedLoopError",
				       state.steeringClosedLoopError);
	frc::SmartDashboard::PutNumber(name + "/WheelClosedLoopError",
				       state.wheelClosedLoopError);
}

void SwerveModule::Set(ModuleVoltageCommand voltageCommand) {
	controlMode = ModuleControlMode::Voltage;
	this->voltageCommand = voltageCommand;
}

void SwerveModule::Set(const frc::SwerveModuleState& closedLoopCommand) {
	controlMode = ModuleControlMode::ClosedLoop;
	this->closedLoopCommand = closedLoopCommand;
}


void SwerveModule::Update(units::second_t timeStep) {
	state = GetState_Int();
	wpiState.angle = state.heading;
	wpiState.speed = state.velocity;

	switch (controlMode) {
		case ModuleControlMode::Voltage:
			steeringMotor.SetVoltage(
			    voltageCommand.steeringVoltage);
			wheelMotor.SetVoltage(voltageCommand.wheelVoltage);
			break;
		case ModuleControlMode::ClosedLoop:
            steeringMotor.Set(ControlMode::Position, GetActualTargetSteeringMotorPos(closedLoopCommand.angle.Radians().to<double>() * MOTOR_PULSE_PER_RADS, state.steeringMotorRawPos));
            wheelMotor.Set(ControlMode::Velocity, closedLoopCommand.speed.to<double>() * MOTOR_PULSE_PER_METER * 0.1);
			break;
		default:
			break;
	}
}

double SwerveModule::GetHeadingFromSteeringMotorPos(double motorPos) {
	return motorPos * RADS_PER_MOTOR_PULSE;
}

double SwerveModule::GetSteeringMotorPosFromHeading(double heading) {
	return heading * MOTOR_PULSE_PER_RADS;
}

double SwerveModule::GetDistanceFromWheelMotorPos(double motorPos){
    return motorPos * METERS_PER_MOTOR_PULSE;
}

double SwerveModule::GetMPSFromWheelMotorVel(double motorVel){
    return wheelMotor.GetSelectedSensorVelocity() * METERS_PER_MOTOR_PULSE * 10;
}

double SwerveModule::GetActualTargetSteeringMotorPos(double desiredPos, double currentPos){
	double unwrappedMotorPos = Utils::InputModulus(currentPos, -MOTOR_PULSE_PER_STEERING_ROTATION/2, MOTOR_PULSE_PER_STEERING_ROTATION/2);


	double error = desiredPos - unwrappedMotorPos ;

	return currentPos + Utils::InputModulus(error, -MOTOR_PULSE_PER_STEERING_ROTATION/2, MOTOR_PULSE_PER_STEERING_ROTATION/2);
	
}
