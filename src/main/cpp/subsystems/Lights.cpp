// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lights.h"
#include "subsystems/CoralSubsystem.h"

Lights::Lights() {
    for (int i = 0; i < DriveConstants::kLEDtotalLength; i = i + 3) {
    //topLEDArray[i].SetRGB(0, 255, 0);
    //0, 133, 202
    //255, 234, 0

    LEDArray[i].SetRGB(2, 5, 97);
    LEDArray[i + 1].SetRGB(0, 133, 202);
    LEDArray[i + 2].SetRGB(252, 186, 3);
    //ledArray[i + 1].SetRGB(0, 133, 202);
    //ledArray[i + 2].SetRGB(3, 8, 143);
  }
  
  ledLoopCount = 9;
  disableLoopCount = 2;
  m_led.SetLength(DriveConstants::kLEDtotalLength);
  m_led.SetData(LEDArray);
  m_led.Start();
}

// This method will be called once per scheduler run
void Lights::Periodic() {
    std::cout << p_coralSubsystem << " p\n";
    std::cout << p_coralSubsystem->haveCoral << " hc\n";
    if(p_coralSubsystem->haveCoral){
        std::cout << "have coral\n";
        for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
            LEDArray[i].SetRGB(201, 5, 255);
        }
    }

    if(p_coralSubsystem->readyToScore){
        for(int i = 0; i < DriveConstants::kLEDtotalLength; i++){
            LEDArray[i].SetRGB(0, 255, 25);
        }
    }
    
   m_led.SetData(LEDArray);
  m_led.Start();
}
