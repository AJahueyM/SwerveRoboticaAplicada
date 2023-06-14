// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once
#include "units/angular_velocity.h"
#include "units/velocity.h"

#include <ctre/Phoenix.h>

#include <frc/kinematics/SwerveModuleState.h>

enum class ModuleControlMode {
    Voltage,
    ClosedLoop
};

struct ModuleVoltageCommand {
    units::volt_t steeringVoltage = 0_V;
    units::volt_t wheelVoltage = 0_V;
};


struct ModuleState {
    frc::Rotation2d heading;
    units::radians_per_second_t headingVelocity = 0_rad_per_s;
    
    units::meter_t distance = 0_m;
    units::meters_per_second_t velocity = 0_mps;

    units::volt_t steeringVoltage = 0_V;
    units::volt_t wheelVoltage = 0_V;

    double wheelMotorRawPos = 0;
    double wheelMotorRawVel = 0;

    double steeringMotorRawPos = 0;
    double steeringMotorRawVel = 0;

    double steeringClosedLoopError = 0;
    double wheelClosedLoopError = 0;
};

class SwerveModule {
 public:
  SwerveModule(int steeringId, int wheelID, int canCoderID, frc::Rotation2d headingOffset);
  const ModuleState& GetState(); //Local to module
  const frc::SwerveModuleState& GetWPIState(); //Local to module

  void PublishTelemetry(std::string name);
  void Set(ModuleVoltageCommand voltageCommand);
  void Set(const frc::SwerveModuleState& closedLoopCommand);
  void Update(units::second_t timeStep = 0.01_s);

 private:
   ModuleState GetState_Int(); //Local to module

    double GetHeadingFromSteeringMotorPos(double motorPos);
    double GetSteeringMotorPosFromHeading(double heading);

    double GetDistanceFromWheelMotorPos(double motorPos);
    double GetMPSFromWheelMotorVel(double motorVel);

    double GetActualTargetSteeringMotorPos(double desiredPos, double currentPos);


    static constexpr double WHEEL_GEAR_RATIO = 6.75;
    static constexpr double WHEEL_RADIUS = 0.0508;
    
    static constexpr double STEERING_GEAR_RATIO = 150.0 / 7.0;
    
    static constexpr double FALCON_PULSE_PER_REV = 2048;


    static constexpr double RADS_PER_MOTOR_PULSE = 2.0 * M_PI / FALCON_PULSE_PER_REV / STEERING_GEAR_RATIO;
    static constexpr double MOTOR_PULSE_PER_RADS = FALCON_PULSE_PER_REV * STEERING_GEAR_RATIO / (2.0 * M_PI);
    static constexpr double MOTOR_PULSE_PER_STEERING_ROTATION = FALCON_PULSE_PER_REV * STEERING_GEAR_RATIO;

    static constexpr double METERS_PER_MOTOR_PULSE = 2.0 * M_PI * WHEEL_RADIUS / FALCON_PULSE_PER_REV / WHEEL_GEAR_RATIO;
    static constexpr double MOTOR_PULSE_PER_METER = 1.0 / METERS_PER_MOTOR_PULSE;

    ModuleVoltageCommand voltageCommand;
    frc::SwerveModuleState closedLoopCommand;
    
    ModuleState state;
    frc::SwerveModuleState wpiState;

    WPI_TalonFX steeringMotor;
    WPI_TalonFX wheelMotor;
    WPI_CANCoder steeringMagnetometer;
    ModuleControlMode controlMode = ModuleControlMode::Voltage;

};
