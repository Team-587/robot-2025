// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralSubsystem.h"
#include "Configs.h"
#include "RobotContainer.h"

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
    
    m_uppiesMotor2.SetInverted(true);
    #endif
}

// This method will be called once per scheduler run
void CoralSubsystem::Periodic() {

    haveCoral = m_houseSwitch.Get();
    bool readyToScore = false;
    bool houseMovingAngle;
    double coDriverRightTrigger = (m_codriverController.GetRightTriggerAxis() * 0.5);

    #ifndef usingNeo
    //STOW
    if(desiredState == STOW){
        m_houseMotor.Set(CoralConstants::kHouseStopSpeed);
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        houseMovingAngle = true;
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesIntakeHeight, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkUppiesHeight(CoralConstants::kUppiesIntakeHeight)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
                houseMovingAngle = false;
                std::cout << "STOW\n";
            }
        }
    }
    //INTAKE
    if(desiredState == INTAKE){
        m_houseMotor.Set(CoralConstants::kHouseStopSpeed);
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesIntakeHeight, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkUppiesHeight(CoralConstants::kUppiesIntakeHeight)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristIntakeAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
                m_houseMotor.Set(CoralConstants::kHouseIntakeSpeed);
                std::cout << "INTAKE\n";
            }
        }
        if(haveCoral == true) {
            std::cout << "We have note now\n";
            m_houseMotor.Set(CoralConstants::kHouseStopSpeed);
        }
    }
    //LEVEL1
    if(desiredState == LEVEL1){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        houseMovingAngle = true;
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL1Height, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkUppiesHeight(CoralConstants::kUppiesL1Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL1Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
                if(checkWristAngle(CoralConstants::kWristL1Angle)){
                    std::cout << "LEVEL 1\n";
                    houseMovingAngle = false;
                    readyToScore = true;
                }else{
                    readyToScore = false;
                }
            }
        }
    }
    //LEVEL2
    if(desiredState == LEVEL2){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        houseMovingAngle = true;
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL2Height, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkUppiesHeight(CoralConstants::kUppiesL2Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL2Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
                if(checkWristAngle(CoralConstants::kWristL2Angle)){
                    std::cout << "LEVEL 2\n";
                    houseMovingAngle = false;
                    readyToScore = true;
                }else{
                    readyToScore = false;
                }
            }
        }
    }
    //LEVEL3 
    if(desiredState == LEVEL3){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        houseMovingAngle = true;
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL3Height, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkUppiesHeight(CoralConstants::kUppiesL3Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL3Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
                if(checkWristAngle(CoralConstants::kWristL3Angle)){
                    std::cout << "LEVEL 3\n";
                    houseMovingAngle = false;
                    readyToScore = true;
                }else{
                    readyToScore = false;
                }
            }
        }
    }
    //LEVEL4
    if(desiredState == LEVEL4){
        m_wristClosedLoopController.SetReference(CoralConstants::kWristMoveAngle, rev::spark::SparkLowLevel::ControlType::kPosition);
        houseMovingAngle = true;
        if(checkWristAngle(CoralConstants::kWristMoveAngle)){
            m_uppies1ClosedLoopController.SetReference(CoralConstants::kUppiesL4Height, rev::spark::SparkLowLevel::ControlType::kPosition);
            if(checkUppiesHeight(CoralConstants::kUppiesL4Height)){
                m_wristClosedLoopController.SetReference(CoralConstants::kWristL4Angle, rev::spark::SparkLowLevel::ControlType::kPosition);
                if(checkWristAngle(CoralConstants::kWristL4Angle)){
                    std::cout << "LEVEL 4\n";
                    houseMovingAngle = false;
                    readyToScore = true;
                }else{
                    readyToScore = false;
                }
            }
        }
    }

    //READY TO SCORE
    if(readyToScore == true){
        std::cout << "READY TO SCORE\n";
        m_houseMotor.Set(coDriverRightTrigger);
        if(haveCoral == false){
            m_houseMotor.Set(CoralConstants::kHouseStopSpeed);
        }
    }else{
        m_houseMotor.Set(CoralConstants::kHouseStopSpeed);
    }
    //MOVING
    if(houseMovingAngle == true){
        std::cout << "Moving\n";
        m_houseMotor.Set(CoralConstants::kHouseStopSpeed);
    }
    #endif
}

void CoralSubsystem::setState(coralState newState) {
    desiredState = newState;
}

bool CoralSubsystem::checkWristAngle(double wristAngle){
    #ifndef usingNeo
    double currentWristAngle = m_wristEncoder.GetPosition();
    if(wristAngle <= (currentWristAngle + 1.0) && wristAngle >= (currentWristAngle - 1.0)){
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