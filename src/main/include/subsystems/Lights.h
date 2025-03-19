// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>

#include "Constants.h"
#include "subsystems/CoralSubsystem.h"

class Lights : public frc2::SubsystemBase {
 public:
  Lights();

  void setCoralSubsystem(CoralSubsystem *tmpcs){p_coralSubsystem = tmpcs;}; 
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  int ledLoopCount;
  int disableLoopCount;

  frc::AddressableLED m_led{DriveConstants::kLEDPort}; 
  std::array<frc::AddressableLED::LEDData, DriveConstants::kLEDtotalLength > LEDArray;

  CoralSubsystem *p_coralSubsystem = NULL;
};
