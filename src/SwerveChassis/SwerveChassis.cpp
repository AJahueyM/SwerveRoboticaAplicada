// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveChassis.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveChassis::SwerveChassis() {
    modules.emplace_back(&topRight);
    modules.emplace_back(&topLeft);
    modules.emplace_back(&bottomRight);
    modules.emplace_back(&bottomLeft);

    navx.Calibrate();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    double startTime = frc::Timer::GetFPGATimestamp().value();
    while (navx.IsCalibrating()) {
        double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
        if (timePassed > 10) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    navx.ZeroYaw();

};

// This method will be called once per scheduler run
void SwerveChassis::Periodic() {
    units::second_t current = frc::Timer::GetFPGATimestamp();
    units::second_t timeStep = current - lastUpdateTime;

    for(size_t i = 0; i < modules.size(); ++i){
        auto& module = modules[i];
        module->Update(timeStep);

        moduleWPIStates[i] = module->GetWPIState();

        modulesPos[i].distance = module->GetState().distance;
        modulesPos[i].angle = moduleWPIStates[i].angle;
    }

    odometry.Update(frc::Rotation2d(units::degree_t(-navx.GetAngle())), modulesPos);

    estimatedPose = odometry.GetEstimatedPosition();


    auto targets = kinematics.ToSwerveModuleStates(targetChassisSpeeds, moduleWPIStates, timeStep);
    // auto targets = wpiKinematics.ToSwerveModuleStates(targetChassisSpeeds);

    // for(size_t i = 0; i < targets.size(); ++i){
    //     targets[i] = frc::SwerveModuleState::Optimize(targets[i], moduleWPIStates[i].angle);
    // }

    wpiKinematics.DesaturateWheelSpeeds(&targets, 3_mps);

    topRight.Set(targets[0]);
    topLeft.Set(targets[1]);
    bottomRight.Set(targets[2]);
    bottomLeft.Set(targets[3]);
    
    PublishTelemetry();

    lastUpdateTime = current;
}

void SwerveChassis::SetChassisSpeeds(const frc::ChassisSpeeds& speeds){
    this->targetChassisSpeeds = speeds;
}

void SwerveChassis::ResetHeading(){
    
    odometry.ResetPosition({units::degree_t(-navx.GetAngle())}, modulesPos, frc::Pose2d(estimatedPose.X(),estimatedPose.Y(), {0_deg}));
}

const wpi::array<frc::SwerveModuleState, 4>& SwerveChassis::GetModuleStates(){
    return moduleWPIStates;
}

const frc::Pose2d& SwerveChassis::GetEstimatedPose(){
    return estimatedPose;
}

frc::ChassisSpeeds SwerveChassis::GetChassisSpeeds(){
    return wpiKinematics.ToChassisSpeeds(moduleWPIStates);
}

void SwerveChassis::PublishTelemetry(){
    topRight.PublishTelemetry("Swerve/TopRight");
    topLeft.PublishTelemetry("Swerve/TopLeft");
    bottomRight.PublishTelemetry("Swerve/BottomRight");
    bottomLeft.PublishTelemetry("Swerve/BottomLeft");

    frc::SmartDashboard::PutNumber("Swerve/Heading", estimatedPose.Rotation().Degrees().to<double>());
    frc::SmartDashboard::PutNumber("Swerve/X", estimatedPose.X().to<double>());
    frc::SmartDashboard::PutNumber("Swerve/Y", estimatedPose.Y().to<double>());

}