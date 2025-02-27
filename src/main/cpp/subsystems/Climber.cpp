// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber() = default;

// This method will be called once per scheduler run
void Climber::Periodic() {
    double coDriverRSY = m_codriverController.GetRightY();
    frc::SmartDashboard::PutBoolean("canClimb", canClimb);
    #ifdef haveClimber
    if(canClimb){
        m_climberMotor.Set(coDriverRSY);
    }else{
        m_climberMotor.Set(0.0);
    }
    #endif
}

void Climber::allowClimb(){
    canClimb = !canClimb;
}
