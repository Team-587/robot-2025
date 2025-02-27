// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#pragma once
#define haveClimber
#define homeField
#define usingNeo


/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
#ifdef usingNeo
constexpr units::meters_per_second_t kMaxSpeed = 5.0_mps;
#else
//constexpr units::meters_per_second_t kMaxSpeed = 5.9_mps;
constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;
#endif
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
#ifdef usingNeo
constexpr units::meter_t kTrackWidth =
    0.496_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.496_m;  // Distance between centers of front and back wheels on robot
#else 
constexpr units::meter_t kTrackWidth =
    0.5468_m;
constexpr units::meter_t kWheelBase =
    0.5468_m;
#endif
// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = 0;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = 0;
constexpr double kRearRightChassisAngularOffset = 0;

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 18;
constexpr int kRearLeftDrivingCanId = 16;
constexpr int kFrontRightDrivingCanId = 11;
constexpr int kRearRightDrivingCanId = 13;

constexpr int kFrontLeftTurningCanId = 17;
constexpr int kRearLeftTurningCanId = 15;
constexpr int kFrontRightTurningCanId = 12;
constexpr int kRearRightTurningCanId = 14;

constexpr int kBallPickupWristCanId = 20;
constexpr int kBallPickupIntakeCanId = 21;
constexpr int kUppies1CanId = 22;
constexpr int kUppies2CanId = 23;
constexpr int kHouseWristCanId = 24;
constexpr int kHouseCanId = 25;
constexpr int kClimberCanId = 26;
constexpr int kClimberCanId2 = 27;

constexpr int kHouseSwitch = 1;
constexpr int kUppiesSwitch = 2;

constexpr int kDistanceSensor = 0;

constexpr int kLEDPort = 0;
constexpr int kLEDtotalLength = 36; //Total LED length should be multiples of 2
}  // namespace DriveConstants

namespace ModuleConstants {
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr double kDrivingMotorPinionTeeth1 = 14.0;
constexpr double kDrivingMotorPinionTeeth2 = 28.0;
constexpr double kDrivingMotorPinionTeeth3 = 15.0;
constexpr double kDrivenMotorPinionTeeth1 = 50.0;
constexpr double kDrivenMotorPinionTeeth2 = 16.0;
constexpr double kDrivenMotorPinionTeeth3 = 45.0;

// Calculations required for driving motor conversion factors and feed forward
#ifdef usingNeo
constexpr double kDrivingMotorFreeSpeedRps =
    (5820.0 / 60);  // NEO free speed is 5820 RPM (As of 11/18/2024)
                    // Vortex free speed is 6784 RPM
#else
constexpr double kDrivingMotorFreeSpeedRps =
    (6784.0 / 60);

#endif

constexpr units::meter_t kWheelDiameter = 0.1016_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction = (kDrivenMotorPinionTeeth1 / kDrivingMotorPinionTeeth1)
         * (kDrivenMotorPinionTeeth2 / kDrivingMotorPinionTeeth2)
         * (kDrivenMotorPinionTeeth3 / kDrivingMotorPinionTeeth3);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;
}  // namespace ModuleConstants

namespace AlgaeConstants {

constexpr double kBallIntakeAngle = 55.0;
constexpr double kBallScoreAngle = 30.0;
constexpr double kBallStowAngle = 5.0;
constexpr double kBallHalfStow = 30.0;

constexpr double kBallMinSpeed = 0.0;
constexpr double kBallMaxSpeed = 0.0;

constexpr double kBallAutoIntakeSpeed = 0.3;
constexpr double kBallAutoScoreSpeed = -0.3;
constexpr double kBallIntakeSpeed = 0.3;
constexpr double kBallScoreSpeed = -0.3;
}

namespace CoralConstants {

    //Elevator Constants
    constexpr double kUppiesIntakeHeight = 0.2;
    constexpr double kUppiesL1Height = 0.2;
    constexpr double kUppiesL2Height = 9.5;
    constexpr double kUppiesL3Height = 17.7;
    constexpr double kUppiesL4Height = 31.5;
    constexpr double kUppiesMaxHeight = 33.0;
    constexpr double kUppiesMinHeight = 0.0;
    constexpr double kAlgaeRemoveHeight1 = 11.0;
    constexpr double kAlgaeRemoveHeight2 = 19.5;
    constexpr double kAlgaeScoreHeight = 3.0;

    //Wrist Constants
    constexpr double kWristIntakeAngle = 85.0;
    constexpr double kWristMoveAngle = 160.0;
    constexpr double kWristL1Angle = 185.0;
    constexpr double kWristL1AngleAuto = 205.0;
    constexpr double kWristL2Angle = 260.0;
    constexpr double kWristL3Angle = 260.0;
    constexpr double kWristL4Angle = 270.0;
    constexpr double kWristMaxAngle = 0.0;
    constexpr double kWristMinAngle = 0.0;
    constexpr double kBallRemoveAngle = 210.0;

    //House Constants
    constexpr double kHouseIntakeSpeed = 0.30;
    constexpr double kHouseL1Speed = -0.13;
    constexpr double kHouseL2Speed = -0.3;
    constexpr double kHouseL3Speed = -0.3;
    constexpr double kHouseL4Speed = -0.3;
    constexpr double kHouseStopSpeed = 0.0;
    constexpr double kHouseMaxSpeed = 0.0;
    constexpr double kScoreSpeed = -0.3;
    constexpr double kBackspin = 0.07;
    constexpr double kRemoveSpeed = -0.20;
    constexpr double kAlgaeShoot = 0.30;
}

namespace AutoConstants {
#ifdef usingNeo
constexpr auto kMaxSpeed = 5_mps; // 5.9 for the vortex and 5 for the neo :p
#else
constexpr auto kMaxSpeed = 5.9_mps;
#endif
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kCoDriverControllerPort = 1;
constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants
