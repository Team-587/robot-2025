// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Configs.h"

Climber::Climber() {}

// This method will be called once per scheduler run
void Climber::Periodic() {
    double coDriverRSY = m_codriverController.GetRightY();
    double coDriverLT = m_codriverController.GetLeftTriggerAxis();
    double coDriverRT = m_codriverController.GetRightTriggerAxis();
    frc::SmartDashboard::PutNumber("CoDriver Right Y", coDriverRSY);
    frc::SmartDashboard::PutBoolean("canClimb", canClimb);
    frc::SmartDashboard::PutNumber("velocity", m_climberMotor.GetAbsoluteEncoder().GetVelocity());
    frc::SmartDashboard::PutBoolean("x Pressed", xIsPressed);
    //#ifdef haveClimber
    if(canClimb == true){
        if(coDriverRT > 0.2) {
            climberIn();
        } else if(coDriverLT > 0.2){
            climberOut();
        } else {
            m_climberMotor.StopMotor();
        }
        if(xIsPressed == true){
            m_climberServo.Set(0);
        }else{
            m_climberServo.Set(0.65);
        }
        //frc::SmartDashboard::PutNumber("SetSpeed", m_climberMotor.Get());
    }else if(canClimb == false){
        m_climberMotor.StopMotor();
    }
    //#endif
}

void Climber::allowClimb(){
    canClimb = !canClimb;
}

void Climber::xPressed() {
    if(canClimb) {
        xIsPressed = !xIsPressed;
    } else {
        xIsPressed = false;
    }
    }

void Climber::dropHopper(){
    if(canClimb){
        m_hopperServo.Set(0);
    }
}

void Climber::climberIn() {
    if(canClimb) {
        m_climberMotor.Set(m_codriverController.GetRightTriggerAxis());
    }
}

void Climber::climberOut() {
    if(canClimb) {
        m_climberMotor.Set(m_codriverController.GetLeftTriggerAxis() * -1.0);
    }
}

bool Climber::climbMode(){
    return canClimb;
}