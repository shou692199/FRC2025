import math
from wpimath.units import inchesToMeters
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from pathplannerlib.config import RobotConfig
from phoenix6 import CANBus

class ModuleConstants:
  kCANbus = CANBus("rio")
  kWheelDiameterMeters = inchesToMeters(4)
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
  kTrackWidth = inchesToMeters(21)
  kWheelBase  = inchesToMeters(21)
  kDriveKinematics = SwerveDrive4Kinematics(
    Translation2d( kWheelBase / 2,  kTrackWidth / 2),
    Translation2d( kWheelBase / 2, -kTrackWidth / 2),
    Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
    Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
  )

  kPhysicalMaxSpeedMetersPerSecond = 5
  kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * math.tau
  kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
  kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4
  kTeleDriveMaxAccelerationUnitsPerSecond = 3.0
  kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0

  kDeadband = 0.06

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

  kRobotConfig = RobotConfig.fromGUISettings()

class AutoConstants:
  kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4
  kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10
  kMaxAccelerationMetersPerSecondSquared = 3.0
  kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4
  kPXController = 0.5
  kPYController = 0.5
  kPThetaController = 3.2
  kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
    kMaxAngularSpeedRadiansPerSecond,
    kMaxAngularAccelerationRadiansPerSecondSquared
  )

class VisionConstants:
  kMainCameraName = "Microsoft_LifeCam_HD-3000"