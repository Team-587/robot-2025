#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

using namespace rev::spark;

namespace Configs {
class MAXSwerveModule {
 public:
  static SparkMaxConfig& DrivingConfig() {
    static SparkMaxConfig drivingConfig{};

    // Use module constants to calculate conversion factors and feed forward
    // gain.
    double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                           std::numbers::pi /
                           ModuleConstants::kDrivingMotorReduction;
    double drivingVelocityFeedForward =
        1 / ModuleConstants::kDriveWheelFreeSpeedRps;

    drivingConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(true)
        .SmartCurrentLimit(50);
    drivingConfig.encoder
        .PositionConversionFactor(drivingFactor)          // meters
        .VelocityConversionFactor(drivingFactor / 60.0);  // meters per second
    drivingConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(0.04, 0, 0)
        .VelocityFF(drivingVelocityFeedForward)
        .Pid(0.08, 0, 0, rev::spark::ClosedLoopSlot::kSlot1)
        .VelocityFF(1 / ModuleConstants::kDriveWheelFreeSpeedRps, rev::spark::ClosedLoopSlot::kSlot1)
        .OutputRange(-1, 1);

    return drivingConfig;
  }

  static SparkMaxConfig& TurningConfig() {
    static SparkMaxConfig turningConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 2 * std::numbers::pi;

    turningConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    turningConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    turningConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return turningConfig;
  }
};

static SparkMaxConfig& wristConfig() {
    static SparkMaxConfig wristConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 360;

    wristConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(60)
        .Inverted(false);
    wristConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // degrees
        .VelocityConversionFactor(turningFactor / 60.0);  // degrees per second
    wristConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        //.Pid(0.003, 0, 0)
        .Pid(0.00350, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(false);
        //.PositionWrappingInputRange(0, turningFactor);

    return wristConfig;
  }

  static SparkMaxConfig& uppiesConfig1() {
    static SparkMaxConfig uppiesConfig1{};

    // Use module constants to calculate conversion factor
    double turningFactor = 1.432 * std::numbers::pi;

    uppiesConfig1.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50)
        .Inverted(true);
    uppiesConfig1
        .alternateEncoder
        .CountsPerRevolution(8192)
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(true)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
          
    uppiesConfig1.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder)
        // These are example gains you may need to them for your own robot!
        //.Pid(.11, 0, 0)
        .Pid(.085, 0, 0)
        //.Pid(0, 0, 0)
        .OutputRange(-0.55, 1);
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        //.PositionWrappingEnabled(true)
        //.PositionWrappingInputRange(0, turningFactor);

    return uppiesConfig1;
  }

  static SparkMaxConfig& uppiesConfig2() {
    static SparkMaxConfig uppiesConfig2{};

    // Use module constants to calculate conversion factor
    //double turningFactor = 2 * std::numbers::pi;

    uppiesConfig2.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .Inverted(false)
        .SmartCurrentLimit(50);
    /*uppiesConfig2
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    uppiesConfig2.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);*/

    uppiesConfig2.Follow(DriveConstants::kUppies1CanId, true);

    return uppiesConfig2;
  }

  static SparkMaxConfig& ballWristConfig() {
    static SparkMaxConfig ballWristConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 360;

    ballWristConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        //.Inverted(true)
        .SmartCurrentLimit(60);
    ballWristConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // degrees
        .VelocityConversionFactor(turningFactor / 60.0);  // degrees per second
    ballWristConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        //.Pid(.2, 0, 0)//.VelocityFF(.001)
        .Pid(0, 0, 0)
        .OutputRange(-1.0, 1.0)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return ballWristConfig;
  }

static SparkMaxConfig& climberConfig2() {
    static SparkMaxConfig climberConfig2{};

    // Use module constants to calculate conversion factor
    //double turningFactor = 2 * std::numbers::pi;

    climberConfig2.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50)
        .Follow(DriveConstants::kClimberCanId, true);

    return climberConfig2;
}

static SparkMaxConfig& climberHopperConfig2() {
    static SparkMaxConfig climberHopperConfig2{};

    // Use module constants to calculate conversion factor
    //double turningFactor = 2 * std::numbers::pi;

    climberHopperConfig2.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50)
        .Follow(DriveConstants::kClimberHopperCanId, true);

    return climberHopperConfig2;
}
  
}  // namespace Configs
