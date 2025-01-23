import math
from enum import Enum
from wpimath.units import inchesToMeters
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from pathplannerlib.config import RobotConfig
from phoenix6 import CANBus

class ModuleConstants:
  kCANbus = CANBus("rio")
  kWheelDiameterMeters = 0.097
  kDriveMotorGearRatio = 1 / 6.75
  kSteerMotorGearRatio = 1 / 12.8
  kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
  kSteerEncoderRot2Rad = kSteerMotorGearRatio * math.tau
  kDriveMotorMaxRPM = 6380
  kDriveMotorMaxVoltage = 12
  kPDriveMotor = 0.8
  kSDriveMotor = 0.04
  kVDriveMotor = kDriveMotorMaxVoltage * 60 / kDriveEncoderRot2Meter / kDriveMotorMaxRPM
  kPSteerMotor = 0.5

class DriveConstants:
  kTrackWidthMeters = 0.7
  kWheelBaseMeters  = 0.53507
  kDriveKinematics = SwerveDrive4Kinematics(
    Translation2d( kWheelBaseMeters / 2,  kWheelBaseMeters / 2),
    Translation2d( kWheelBaseMeters / 2, -kWheelBaseMeters / 2),
    Translation2d(-kWheelBaseMeters / 2,  kWheelBaseMeters / 2),
    Translation2d(-kWheelBaseMeters / 2, -kWheelBaseMeters / 2)
  )

  kPhysicalMaxSpeedMetersPerSecond = 5
  kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * math.tau
  kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
  kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4
  kTeleDriveMaxAccelerationUnitsPerSecond = 3.0
  kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0

  kFrontLeftDriveMotorId = 0
  kFrontLeftSteerMotorId = 1
  kFrontLeftShaftEncoderId = 10
  kFrontLeftShaftEncoderOffset = 0.903

  kFrontRightDriveMotorId = 2
  kFrontRightSteerMotorId = 3
  kFrontRightShaftEncoderId = 11
  kFrontRightShaftEncoderOffset = 0.9929

  kBackLeftDriveMotorId = 6
  kBackLeftSteerMotorId = 7
  kBackLeftShaftEncoderId = 13
  kBackLeftShaftEncoderOffset = 0.488

  kBackRightDriveMotorId = 4
  kBackRightSteerMotorId = 5
  kBackRightShaftEncoderId = 12
  kBackRightShaftEncoderOffset = 0.1413

class ElevatorConstants:
  kChainPitchMeters = inchesToMeters(0.25)
  kChainWheelTeeth = 26
  kLiftMotorGearRatio = 1 / 20
  kLiftEncoderRot2Meter = kLiftMotorGearRatio * kChainWheelTeeth * kChainPitchMeters * 2

  kSmartCurrentLimit = 50
  kForwardLimitMeters = 1.3
  kReverseLimitMeters = 0
  kTeleMaxDutyCycle = 0.3
  kAutoMaxDutyCycle = 0.65
  kPLiftMotor = 20

  kLiftMotorId = 14
  kLiftMotorSubId = 15

class ReefLayers(Enum):
  L1 = 0.20
  L2 = 0.55
  L3 = 0.90
  L4 = 1.25

class AutoConstants:
  kRobotConfig = RobotConfig.fromGUISettings()
  kPTranslation = 0.5
  kPRotation = 3.2

class OIConstants:
  kDriverControllerPort = 0
  kDeadband = 0.06

class VisionConstants:
  kMainCameraName = "Microsoft_LifeCam_HD-3000"