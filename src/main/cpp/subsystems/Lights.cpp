// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lights.h"
#include "subsystems/CoralSubsystem.h"

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
    //std::cout << p_coralSubsystem << " p\n";
    //std::cout << p_coralSubsystem->haveCoral << " hc\n";
    if(p_coralSubsystem->haveCoral){
        std::cout << "LIGHTS have coral\n";
        for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
            LEDArray[i].SetRGB(201, 5, 255);
        }
    }
    else{
        //std::cout << "Don't have coral\n";
        for (int j = 0; j < DriveConstants::kLEDtotalLength - 1; j = j + 2) {
            LEDArray[j].SetRGB(0, 133, 202);
            LEDArray[j + 1].SetRGB(252, 186, 3);
        }
    }
    if(p_coralSubsystem->readyToScore){
        for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
            LEDArray[i].SetRGB(0, 255, 25);
        }
    } else if(p_coralSubsystem->readyToScore == false && p_coralSubsystem->haveCoral == false){
        for (int j = 0; j < DriveConstants::kLEDtotalLength - 1; j = j + 2) {
            LEDArray[j].SetRGB(0, 133, 202);
            LEDArray[j + 1].SetRGB(252, 186, 3);
        }
    }
    
   m_led.SetData(LEDArray);
  m_led.Start();
}
