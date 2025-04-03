// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lights.h"
#include "subsystems/CoralSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

Lights::Lights() {
    for (int j = 0; j < DriveConstants::kLEDtotalLength - 1; j = j + 2) {
            LEDArray[j].SetRGB(0, 133, 202);
            LEDArray[j + 1].SetRGB(252, 186, 3);
        }
  
  ledLoopCount = 9;
  disableLoopCount = 2;
  m_led.SetLength(DriveConstants::kLEDtotalLength);
  m_led.SetData(LEDArray);
  m_led.Start();
}

// This method will be called once per scheduler run
void Lights::Periodic() {
    std::shared_ptr<nt::NetworkTable> tableLeft = nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");
    double idLeft = tableLeft->GetNumber("tid", 0.0);
    std::shared_ptr<nt::NetworkTable> tableRight = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
    double idRight = tableRight->GetNumber("tid", 0.0);

    frc::SmartDashboard::PutBoolean("lights climb", climbModeActivated);
    
    if(leftAutoAlign){
        if(idLeft == 18 || idLeft == 20 || idLeft == 22 || idLeft == 9 || idLeft == 11 || idLeft == 7){
            for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
                LEDArray[i].SetRGB(0, 255, 0);
            }
        }else if(idLeft == 17 || idLeft == 19 || idLeft == 21 || idLeft == 10 || idLeft == 6 || idLeft == 8){
            for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
                LEDArray[i].SetRGB(255, 20, 0);
            }
        }
    }else if(rightAutoAlign){
        if(idRight == 18 || idRight == 20 || idRight == 22 || idRight == 9 || idRight == 11 || idRight == 7){
            for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
                LEDArray[i].SetRGB(0, 255, 0);
            }
        }else if(idRight == 17 || idRight == 19 || idRight == 21 || idRight == 10 || idRight == 6 || idRight == 8){
            for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
                LEDArray[i].SetRGB(255, 20, 0);
            }
        }
    }

    if(p_coralSubsystem->haveCoral && rightAutoAlign == false && leftAutoAlign == false && climbModeActivated == false){
        //std::cout << "LIGHTS have coral\n";
        for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
            LEDArray[i].SetRGB(201, 5, 255);
        }
    }

    if(climbModeActivated && rightAutoAlign == false && leftAutoAlign == false && p_coralSubsystem->haveCoral == false){
        for(int i = 0; i < DriveConstants::kLEDtotalLength - 9; i = i + 12){
            LEDArray[i].SetRGB(255, 0, 0);
            LEDArray[i + 1].SetRGB(255, 0, 0);
            LEDArray[i + 2].SetRGB(255, 20, 0);
            LEDArray[i + 3].SetRGB(255, 20, 0);
            LEDArray[i + 4].SetRGB(100, 20, 0);
            LEDArray[i + 5].SetRGB(100, 20, 0);
            LEDArray[i + 6].SetRGB(0, 255, 0);
            LEDArray[i + 7].SetRGB(0, 255, 0);
            LEDArray[i + 8].SetRGB(0, 0, 255);
            LEDArray[i + 9].SetRGB(0, 0, 255);
            LEDArray[i + 10].SetRGB(201, 5, 255);
            LEDArray[i + 11].SetRGB(201, 5, 255);
        }
    }else if(p_coralSubsystem->haveCoral == false && rightAutoAlign == false && leftAutoAlign == false && climbModeActivated == false){
        //std::cout << "Don't have coral\n";
        for (int j = 0; j < DriveConstants::kLEDtotalLength - 1; j = j + 2) {
            LEDArray[j].SetRGB(0, 133, 202);
            LEDArray[j + 1].SetRGB(252, 186, 3);
        }
    }
    m_led.SetData(LEDArray);
    m_led.Start();
}

void Lights::autoAlignLeft(bool left){
    leftAutoAlign = left;
}

void Lights::autoAlignRight(bool right){
    rightAutoAlign = right;
}

void Lights::climbMode(){
    climbModeActivated = !climbModeActivated;
}
