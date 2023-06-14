// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveKinematics.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include "Utils.h"

SwerveKinematics::SwerveKinematics(const wpi::array<frc::Translation2d, 4>& modulePos){
    params.topRightTranslation = modulePos[0];
    params.topLeftTranslation = modulePos[1];
    params.bottomRightTranslation = modulePos[2];
    params.bottomLeftTranslation = modulePos[3];

    A <<    1, 0, -params.topRightTranslation.Y().to<double>(),
            0, 1, params.topRightTranslation.X().to<double>(),
            1, 0, -params.topLeftTranslation.Y().to<double>(),
            0, 1, params.topLeftTranslation.X().to<double>(),
            1, 0, -params.bottomRightTranslation.Y().to<double>(),
            0, 1, params.bottomRightTranslation.X().to<double>(),
            1, 0, -params.bottomLeftTranslation.Y().to<double>(),
            0, 1, params.bottomLeftTranslation.X().to<double>();

};


wpi::array<frc::SwerveModuleState, 4> SwerveKinematics::ToSwerveModuleStates(const frc::ChassisSpeeds& chassisSpeeds, const wpi::array<frc::SwerveModuleState, 4>& currentStates,  units::second_t timestep) {
    chassisSpeedsM <<   chassisSpeeds.vx.to<double>(),
                        chassisSpeeds.vy.to<double>(),
                        chassisSpeeds.omega.to<double>();

    moduleSpeeds = A * chassisSpeedsM;
    
    moduleAccels = (moduleSpeeds - moduleSpeedsPrev) / timestep.to<double>();

    InverseKinematicCommand topRightCmd = GetModuleCommand(moduleSpeeds(0), moduleSpeeds(1), moduleAccels(0), moduleAccels(1));
    InverseKinematicCommand topLeftCmd = GetModuleCommand(moduleSpeeds(2), moduleSpeeds(3), moduleAccels(2), moduleAccels(3));
    InverseKinematicCommand bottomRightCmd = GetModuleCommand(moduleSpeeds(4), moduleSpeeds(5), moduleAccels(4), moduleAccels(5));
    InverseKinematicCommand bottomLeftCmd = GetModuleCommand(moduleSpeeds(6), moduleSpeeds(7), moduleAccels(6), moduleAccels(7));


    moduleSpeedsPrev = moduleSpeeds;


    targetStates[0].angle = {topRightCmd.heading + topRightCmd.headingVelocity * timestep};
    targetStates[0].speed = topRightCmd.wheelVelocity + topRightCmd.wheelAcceleration * timestep;

    targetStates[1].angle = {topLeftCmd.heading + topLeftCmd.headingVelocity * timestep};
    targetStates[1].speed = topLeftCmd.wheelVelocity + topLeftCmd.wheelAcceleration * timestep;
    
    targetStates[2].angle = {bottomRightCmd.heading + bottomRightCmd.headingVelocity * timestep};
    targetStates[2].speed = bottomRightCmd.wheelVelocity + bottomRightCmd.wheelAcceleration * timestep;

    targetStates[3].angle = {bottomLeftCmd.heading + bottomLeftCmd.headingVelocity * timestep};
    targetStates[3].speed = bottomLeftCmd.wheelVelocity + bottomLeftCmd.wheelAcceleration * timestep;


    for(size_t i = 0; i < targetStates.size(); ++i){
        targetStates[i] = frc::SwerveModuleState::Optimize(targetStates[i], currentStates[i].angle);
    }

    return targetStates;
}


    InverseKinematicCommand SwerveKinematics::GetModuleCommand(const double v_mx, const double v_my, const double a_mx, const double a_my){
        InverseKinematicCommand res;

        res.heading = units::radian_t(atan2(v_my, v_mx));
        res.wheelVelocity = units::meters_per_second_t(sqrt(pow(v_mx, 2) + pow(v_my, 2)));

        double denom = pow(v_mx, 2) + pow(v_my, 2);
        if(denom != 0){
            res.wheelAcceleration = units::meters_per_second_squared_t((v_mx * a_mx + v_my * a_my) / (sqrt(denom)));
            res.headingVelocity = units::radians_per_second_t((v_mx * a_my - v_my * a_mx) / denom) ;
        }


        return res;
    }
