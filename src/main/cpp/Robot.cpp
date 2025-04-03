// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/CoralSubsystem.h"
#include <frc/PowerDistribution.h>


void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); 

int pov = m_driverController.GetPOV();
int povCoDriver = m_codriverController.GetPOV();
double coDriverLT = m_codriverController.GetLeftTriggerAxis();
int prevPov;
frc::SmartDashboard::PutNumber("Match Time", (double)frc::DriverStation::GetMatchTime());
//std::cout << pov << "\n";

    /*if(pov == 0){
        m_container.m_ballSubsystem.setState(BallSubsystem::INTAKE);
        //std::cout << "ALGAE INTAKE\n";
        //ALGAE INTAKE
    }else if(pov == 180){
        m_container.m_ballSubsystem.setState(BallSubsystem::SCORE);
        //std::cout << "ALGAE SCORE\n";
        //ALGAE SCORE
    }else if(pov == -1 && m_container.m_ballSubsystem.CurrentState == BallSubsystem::SCORE){
        m_container.m_ballSubsystem.setState(BallSubsystem::STOW);
        //std::cout << "ALGAE STOW\n";
        //ALGAE STOW
    } else if(pov == -1 && m_container.m_ballSubsystem.CurrentState == BallSubsystem::INTAKE) {
      m_container.m_ballSubsystem.setState(BallSubsystem::HALFSTOW);
    }*/

    /*if(povCoDriver == 270) {
        m_container.m_coralSubsystem.setState(CoralSubsystem::ALGAE1);
        prevPov = 270;
    } else if(povCoDriver == 90) {
        m_container.m_coralSubsystem.setState(CoralSubsystem::ALGAE2);
        prevPov = 90;
    } else if((povCoDriver == -1 && AlgaeIntaking1) || (povCoDriver == 180 && AlgaeIntaking1) || (povCoDriver == 0 && AlgaeIntaking1)){
      m_container.m_coralSubsystem.setState(CoralSubsystem::ALGAE1);
    } else if((povCoDriver == -1 && AlgaeIntaking2) || (povCoDriver == 180 && AlgaeIntaking2) || (povCoDriver == 0 && AlgaeIntaking2)){
      m_container.m_coralSubsystem.setState(CoralSubsystem::ALGAE2);
    }*/
    /*else if(povCoDriver == -1 && prevPov == 270) {
        m_container.m_coralSubsystem.setState(CoralSubsystem::ALGAE1);
    } else if(povCoDriver == -1 && prevPov == 90) {
        m_container.m_coralSubsystem.setState(CoralSubsystem::ALGAE2);
    }*/

    /*else if(povCoDriver == 0){
        m_container.m_ballSubsystem.setState(BallSubsystem::INTAKE);
        //std::cout << "ALGAE INTAKE\n";
        //ALGAE INTAKE
    }else if(povCoDriver == 180){
        m_container.m_ballSubsystem.setState(BallSubsystem::SCORE);
        //std::cout << "ALGAE SCORE\n";
        //ALGAE SCORE
    }else if(povCoDriver == -1 && m_container.m_ballSubsystem.CurrentState == BallSubsystem::SCORE){
        m_container.m_ballSubsystem.setState(BallSubsystem::STOW);
        //std::cout << "ALGAE STOW\n";
        //ALGAE STOW
    } else if(povCoDriver == -1 && m_container.m_ballSubsystem.CurrentState == BallSubsystem::INTAKE) {
      m_container.m_ballSubsystem.setState(BallSubsystem::HALFSTOW);
    }*/

    // if(coDriverLT > 0.3){
    //   m_container.m_coralSubsystem.setState(CoralSubsystem::STOW);
    // }

}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
