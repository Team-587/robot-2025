// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Configs.h"

Climber::Climber() {
    #ifdef haveClimber
    m_climberMotor2.Configure(Configs::climberConfig2(), SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    m_climberHopperMotor2.Configure(Configs::climberHopperConfig2(), SparkBase::ResetMode::kResetSafeParameters, SparkBase::PersistMode::kPersistParameters);
    #endif
}

// This method will be called once per scheduler run
void Climber::Periodic() {
    double coDriverRSY = m_codriverController.GetRightY();
    double coDriverLSY = m_codriverController.GetLeftY();
    frc::SmartDashboard::PutBoolean("canClimb", canClimb);
    #ifdef haveClimber
    if(canClimb){
        m_climberMotor.Set(coDriverRSY);
        m_climberHopperMotor.Set(coDriverLSY);
    }else{
        m_climberMotor.Set(0.0);
        m_climberHopperMotor.Set(0.0);
    }
    #endif
}

void Climber::allowClimb(){
    canClimb = !canClimb;
}
