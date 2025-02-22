// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"
#include "Configs.h"
#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>

CoralSubsystem::CoralSubsystem(){
    
    m_wristMotor.Configure(Configs::wristConfig(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
    #ifndef usingNeo
    m_uppiesMotor1.Configure(Configs::uppiesConfig1(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
    m_uppiesMotor2.Configure(Configs::uppiesConfig2(),
                           SparkBase::ResetMode::kResetSafeParameters,
                           SparkBase::PersistMode::kPersistParameters);
    
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

    frc::SmartDashboard::PutNumber("Distance", distance);
    frc::SmartDashboard::PutNumber("Angle", m_wristEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Height", m_uppiesEncoder1.GetPosition());
    
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
        std::cout << "STOW\n";
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
            m_wristClosedLoopController.SetReference(CoralConstants::kWristIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
            std::cout << "STOW DONE\n";
        }
    }

    if(coDriverLT > 0.3 && haveCoral == false){
      m_houseMotor.Set(CoralConstants::kHouseIntakeSpeed);
    }else{
        m_houseMotor.Set(CoralConstants::kBackspin);
        if(haveCoral == true) {
            //houseMovingAngle = true;
            //sleep(10);
            std::cout << "House Stopped\n";
            m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    
    /*if(desiredState == INTAKE){
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
    }*/
    
    //LEVEL1
    if(desiredState == LEVEL1){
        std::cout << "LEVEL 1\n";
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL1Height, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kUppiesL1Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL1Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }

    //LEVEL2
    if(desiredState == LEVEL2){
        std::cout << "LEVEL 2\n";
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL2Height, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kUppiesL2Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL2Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }

    //LEVEL3 
    if(desiredState == LEVEL3){
        std::cout << "LEVEL 3\n";
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL3Height, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
        if(checkUppiesHeight(CoralConstants::kUppiesL3Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL3Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    //LEVEL4
    if(desiredState == LEVEL4){
        std::cout << "LEVEL 4\n";
         m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        // houseMovingAngle = true;
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL4Height, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
         if(checkUppiesHeight(CoralConstants::kUppiesL4Height)){
                 m_wristClosedLoopController.SetReference(CoralConstants::kWristL4Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
        }
    }
    if(m_codriverController.GetRightBumper()){
        if(desiredState == LEVEL1){
            m_houseMotor.Set(CoralConstants::kHouseL1Speed);
        }else if(desiredState == LEVEL2 || desiredState == LEVEL3 || desiredState == LEVEL4){
            m_houseMotor.Set(CoralConstants::kScoreSpeed);
        }else if(desiredState == STOW || desiredState == INTAKE){
            m_houseMotor.Set(CoralConstants::kBackspin);
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
    m_houseMotor.Set(speed);
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
    if(uppiesHeight <= (currentUppiesHeight + 1.0) && uppiesHeight >= (currentUppiesHeight - 1.0)){
        return true;
    }else{
        return false;
    }
    #else
    return true;
    #endif
}