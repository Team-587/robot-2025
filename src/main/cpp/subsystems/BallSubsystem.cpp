// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/BallSubsystem.h"
#include <iostream>

BallSubsystem::BallSubsystem(){
    m_ballWristMotor.Configure(Configs::ballWristConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
    m_ballMotor.SetInverted(true);
}

// This method will be called once per scheduler run
void BallSubsystem::Periodic() {
    double coRightStickVal = m_codriverController.GetRightY();
    //std::cout << m_ballAbsoluteEncoder.GetPosition();    
    bool canRun = false;
    if(DesiredState == STOW) {
        m_ballMotor.Set(AlgaeConstants::kBallMinSpeed);
        m_ballClosedLoopController.SetReference(AlgaeConstants::kBallStowAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        //m_ballMotor.Set(AlgaeConstants::kBallMinSpeed);  
        //canRun = false;
        CurrentState = STOW;
        //std::cout << "ALGAE Stow\n";
    }

    if(DesiredState == HALFSTOW) {
        m_ballMotor.Set(AlgaeConstants::kBallMinSpeed);
        m_ballClosedLoopController.SetReference(AlgaeConstants::kBallHalfStow, rev::spark::SparkLowLevel::ControlType::kPosition);
        //std::cout << "ALGAE Half Stow\n";
        CurrentState = HALFSTOW;

    }

    if(DesiredState == INTAKE) {
        m_ballClosedLoopController.SetReference(AlgaeConstants::kBallIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        m_ballMotor.Set(AlgaeConstants::kBallIntakeSpeed);
        //m_ballMotor.Set(0.0);
        CurrentState = INTAKE;
        //canRun = true;
        //std::cout << "ALGAE Intake\n";
    }

    if(DesiredState == SCORE) {
        m_ballClosedLoopController.SetReference(AlgaeConstants::kBallScoreAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        m_ballMotor.Set(AlgaeConstants::kBallScoreSpeed);
        //m_ballMotor.Set(0.0);
        CurrentState = SCORE;
        //std::cout << "ALGAE Score\n";
        //canRun = true;
    }

    /*if(canRun){
        m_ballMotor.Set(coRightStickVal);
    }else{
        m_ballMotor.Set(AlgaeConstants::kBallMinSpeed);
    }
    */
}

void BallSubsystem::setState(ballStates newState) {
    //std::cout << "setstate " << newState << "\n";
    DesiredState = newState;
}


