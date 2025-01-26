// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/BallSubsystem.h"

BallSubsystem::BallSubsystem(){
    #ifndef usingNeo
    m_ballWristMotor.Configure(Configs::ballWristConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
    #endif
}

// This method will be called once per scheduler run
void BallSubsystem::Periodic() {
    double coRightStickVal = m_codriverController.GetRightY();

    bool canRun = false;
    #ifndef usingNeo
    if(DesiredState == STOW) {
        m_ballClosedLoopController.SetReference(AlgaeConstants::kBallStowAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        m_ballMotor.Set(AlgaeConstants::kBallMinSpeed);  
        canRun = false;
    }

    if(DesiredState == INTAKE) {
        m_ballClosedLoopController.SetReference(AlgaeConstants::kBallIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        canRun = true;
    }

    if(DesiredState == SCORE) {
        m_ballClosedLoopController.SetReference(AlgaeConstants::kBallScoreAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        canRun = true;
    }

    if(canRun){
        m_ballMotor.Set(coRightStickVal);
    }else{
        m_ballMotor.Set(AlgaeConstants::kBallMinSpeed);
    }
    #endif
}

void BallSubsystem::setState(ballStates newState) {
    DesiredState = newState;
}


