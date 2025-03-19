// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"
#include "Configs.h"
#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Robot.h"

CoralSubsystem::CoralSubsystem(){
    #ifndef usingNeo
    m_wristMotor.Configure(Configs::wristConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);

    m_uppiesMotor1.Configure(Configs::uppiesConfig1(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
    m_uppiesMotor2.Configure(Configs::uppiesConfig2(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
    
    autoSpeed = 0.0;
    
    //m_uppiesMotor2.SetInverted(true);
    #endif
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {

    haveCoral = !m_houseSwitch.Get();
    //std::cout << haveCoral << "\n";
    bool readyToScore = false;
    double coDriverRightTrigger = (m_codriverController.GetRightTriggerAxis() * 0.5);
    int distance = m_distanceSensor.GetValue();
    double coDriverLT = m_codriverController.GetLeftTriggerAxis();
    double coDriverRT = m_codriverController.GetRightTriggerAxis();

    frc::SmartDashboard::PutNumber("Distance", distance);
    frc::SmartDashboard::PutBoolean("Have Coral", haveCoral);
    #ifndef usingNeo
    frc::SmartDashboard::PutNumber("Angle", m_wristEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Height", m_uppiesEncoder1.GetPosition());
    #endif
    
    /*if(desiredState == LEVEL1){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristL1Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        m_houseMotor.Set(0.0);
    }else if(desiredState == LEVEL2){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristL2Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        m_houseMotor.Set(0.0);
    }else if(desiredState == LEVEL3){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristL3Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        m_houseMotor.Set(0.0);
    }else if(desiredState == LEVEL4){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        m_houseMotor.Set(0.0);
    }else if(desiredState == INTAKE){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        if(m_houseSwitch.Get() == false) {
            m_houseMotor.Set(0.0);
        } else if(m_houseSwitch.Get() == true){
            m_houseMotor.Set(CoralConstants::kHouseIntakeSpeed);
        }
        
    }*/
    

    #ifndef usingNeo

    /*if(desiredState == LEVEL1){
        m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL1Height, rev::spark::SparkLowLevel::ControlType::kPosition);
    }else if(desiredState == LEVEL2){
        m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL2Height, rev::spark::SparkLowLevel::ControlType::kPosition);
    }else if(desiredState == LEVEL3){
        m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL3Height, rev::spark::SparkLowLevel::ControlType::kPosition);
    }else if(desiredState == LEVEL4){
    m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL4Height, rev::spark::SparkLowLevel::ControlType::kPosition);
    }else if(desiredState == INTAKE){
    m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesIntakeHeight, rev::spark::SparkLowLevel::ControlType::kPosition);
    }*/

    //STOW
    if(desiredState == STOW){
        //std::cout << "STOW\n";
        m_houseMotor.Set(CoralConstants::kBackspin);
        if(haveCoral == true){
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkWristAngle(CoralConstants::kWristMoveAngle)){
                m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesIntakeHeight, rev::spark::SparkLowLevel::ControlType::kPosition);    
            }
        }else{
             m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesIntakeHeight, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kUppiesIntakeHeight)){
            if(haveCoral){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            }else{
                m_wristClosedLoopController.SetReference(CoralConstants::kWristIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            }
        }
    }

    if(frc::DriverStation::IsAutonomousEnabled()){
        std::cout << "Autonomous" << autoSpeed << "\n";
        if(haveCoral == true && autoSpeed >= 0.0){
            m_houseMotor.Set(CoralConstants::kBackspin);
        }else{
            m_houseMotor.Set(autoSpeed);
        }
    }
    else if(coDriverLT > 0.3 && haveCoral == false){
      m_houseMotor.Set(CoralConstants::kHouseIntakeSpeed);
    }else{
        m_houseMotor.Set(CoralConstants::kBackspin);
        if(haveCoral == true) {
            //houseMovingAngle = true;
            //sleep(10);
            std::cout << "House Stopped\n";
            //m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    
    if(desiredState == INTAKE){
        if(haveCoral == true) {
            std::cout << "STOWCORAL\n";
            //houseMovingAngle = true;
            m_houseMotor.Set(CoralConstants::kHouseStopSpeed + 0.05);
            //sleep(10);
            std::cout << "House Stopped\n";
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }

        else if(haveCoral == false) {
        std::cout << "INTAKE\n";

        if(checkWristAngle(CoralConstants::kWristMoveAngle) == true){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesIntakeHeight, rev::spark::SparkLowLevel::ControlType::kPosition);
        
        }

        if(checkUppiesHeight(CoralConstants::kUppiesIntakeHeight) == true){
            m_wristClosedLoopController.SetReference(CoralConstants::kWristIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
                m_houseMotor.Set(CoralConstants::kHouseIntakeSpeed);
         
        } else {
            m_houseMotor.Set(CoralConstants::kHouseStopSpeed);
            if(haveCoral == true){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            }
        }
        }
    }
    
    //LEVEL1
    if(desiredState == LEVEL1){
        std::cout << "LEVEL 1\n";
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL1Height, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kUppiesL1Height)){
            if(frc::DriverStation::IsAutonomousEnabled()){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL1AngleAuto, rev::spark::SparkLowLevel::ControlType::kPosition);
            }else{
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL1Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
            }
        }else{
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }

    //LEVEL2
    if(desiredState == LEVEL2){
        std::cout << "LEVEL 2\n";
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL2Height, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kUppiesL2Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL2Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }else{
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }

    //LEVEL3 
    if(desiredState == LEVEL3){
        std::cout << "LEVEL 3\n";
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL3Height, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kUppiesL3Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL3Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }else{
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    //LEVEL4
    if(desiredState == LEVEL4){
        if(frc::DriverStation::IsAutonomousEnabled()){
            std::cout << "LEVEL 4\n";
            // houseMovingAngle = true;
            m_wristClosedLoopController.SetReference(CoralConstants::kWristL4Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkWristAngle(CoralConstants::kWristL4Angle)){
                m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL4Height, rev::spark::SparkLowLevel::ControlType::kPosition);
            }
        }else if(frc::DriverStation::IsTeleopEnabled()){
            if(checkWristAngle(CoralConstants::kWristMoveAngle)){
                m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL4Height, rev::spark::SparkLowLevel::ControlType::kPosition);
            }
            if(checkUppiesHeight(CoralConstants::kUppiesL4Height)){
                 m_wristClosedLoopController.SetReference(CoralConstants::kWristL4Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
            }else{
                m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            }
        }
    }

    if(desiredState == ALGAE1) {
        if(checkWristAngle(CoralConstants::kWristMoveAngle)) {
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kAlgaeRemoveHeight1, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kAlgaeRemoveHeight1)) {
            m_wristClosedLoopController.SetReference(CoralConstants::kBallRemoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            m_houseMotor.Set(CoralConstants::kRemoveSpeed);
        }else{
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    if(desiredState == ALGAE2) {
        if(checkWristAngle(CoralConstants::kWristMoveAngle)) {
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kAlgaeRemoveHeight2, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kAlgaeRemoveHeight2)) {
            m_wristClosedLoopController.SetReference(CoralConstants::kBallRemoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            m_houseMotor.Set(CoralConstants::kRemoveSpeed);
        }else{
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        } 
    }

    if(desiredState == ALGAESCORE){
        std::cout << "Algae Score\n";
        m_houseMotor.Set(CoralConstants::kRemoveSpeed + 0.05);
        m_uppies1ClosedLoopController.SetReference(CoralConstants::kAlgaeScoreHeight, rev::spark::SparkLowLevel::ControlType::kPosition);
        if(checkUppiesHeight(CoralConstants::kAlgaeScoreHeight)) {
            m_wristClosedLoopController.SetReference(CoralConstants::kBallRemoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        } 
    }

    if(coDriverRT > 0.3){
        if(desiredState == LEVEL1){
            m_houseMotor.Set(CoralConstants::kHouseL1Speed);
        }else if(desiredState == LEVEL2 || desiredState == LEVEL3 || desiredState == LEVEL4){
            m_houseMotor.Set(CoralConstants::kScoreSpeed);
        }else if(desiredState == STOW || desiredState == INTAKE){
            m_houseMotor.Set(CoralConstants::kBackspin);
        }else if(desiredState == ALGAE1 || desiredState == ALGAE2 || desiredState == ALGAESCORE){
            m_houseMotor.Set(CoralConstants::kAlgaeShoot);
        }
    }
    //double coDriverRSY = m_codriverController.GetRightY();
    //m_uppiesMotor.Set(coDriverRSY);

    #endif
}

void CoralSubsystem::setState(coralState newState) {
    std::cout<< "Setting state " << newState << "\n";
    desiredState = newState;
}

void CoralSubsystem::setSpeed(double speed){
    autoSpeed = speed;
}

bool CoralSubsystem::checkWristAngle(double wristAngle){
    #ifndef usingNeo
    double currentWristAngle = m_wristEncoder.GetPosition();
    if(wristAngle <= (currentWristAngle + 15.0) && wristAngle >= (currentWristAngle - 15.0)){
        return true;
    }else{
        return false;
    }
    #else
    return true;
    #endif
}

bool CoralSubsystem::checkUppiesHeight(double uppiesHeight){
    #ifndef usingNeo
    double currentUppiesHeight = m_uppiesEncoder1.GetPosition();
    if(uppiesHeight <= (currentUppiesHeight + 2.0) && uppiesHeight >= (currentUppiesHeight - 2.0)){
        return true;
    }else{
        return false;
    }
    #else
    return true;
    #endif
}