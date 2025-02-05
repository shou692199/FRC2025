import math
from enum import Enum
from wpimath.units import inchesToMeters, degreesToRadians
from wpimath.geometry import Translation2d, Transform3d, Rotation3d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.kinematics import SwerveDrive4Kinematics
from pathplannerlib.config import RobotConfig
from photonlibpy.photonCamera import PhotonCamera
from phoenix6 import CANBus

class ModuleConstants:
  kCANbus = CANBus("canivore")
  kWheelDiameterMeters = 0.097
  kDriveMotorGearRatio = 1 / 6.75
  kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
  kDriveMotorMaxRPM = 6380
  kDriveMotorMaxVoltage = 12
  kPDriveMotor = 0.8
  kSDriveMotor = 0.04
  kVDriveMotor = kDriveMotorMaxVoltage * 60 / kDriveEncoderRot2Meter / kDriveMotorMaxRPM
  kPSteerMotor = 0.7

class DriveConstants:
  kTrackWidthMeters = 0.7
  kWheelBaseMeters  = 0.53507
  kDriveKinematics = SwerveDrive4Kinematics(
    Translation2d( kWheelBaseMeters / 2,  kWheelBaseMeters / 2),
    Translation2d( kWheelBaseMeters / 2, -kWheelBaseMeters / 2),
    Translation2d(-kWheelBaseMeters / 2,  kWheelBaseMeters / 2),
    Translation2d(-kWheelBaseMeters / 2, -kWheelBaseMeters / 2)
  )

  kDeadband = 0.001
  kPHeading = 1.2

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

class ChaseTagConstants:
  kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2
  kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5
  kMaxAccelerationMetersPerSecondSquared = 3
  kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 2
  kPXController = 0.6
  kPYController = 0.6
  kPOController = 1.5

  kXControllerConstraints = TrapezoidProfile.Constraints(
    kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared
  )
  kYControllerConstraints = TrapezoidProfile.Constraints(
    kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared
  )
  kOControllerConstraints = TrapezoidProfileRadians.Constraints(
    kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared
  )

  kXToleranceMeters = 0.1
  kYToleranceMeters = 0.1
  kOToleranceRadians = degreesToRadians(3)

  kTag2GoalTransform = Transform3d(0.5, 0, 0, Rotation3d(0, 0, math.pi))

class OIConstants:
  kDriverControllerPort = 0
  kDeadband = 0.06

class VisionConstants:
  kMainCameraPhoton = PhotonCamera("Microsoft_LifeCam_HD-3000")
  kMainCameraTransform = Transform3d(
    0.35, 0.11, 0.115, Rotation3d(0, degreesToRadians(15), 0)
  )
  kStateStdDevs = (0.05, 0.05, degreesToRadians(5))
  kVisionMesurementStdDevs = (0.5, 0.5, degreesToRadians(30))

class PhysicsConstants:
  kDriveMotorMOI = 0.0001
  kSteerMotorMOI = 0.0001
  kGyroSimDevice = "navX-Sensor[4]"
  kElevatorMassKilograms = 4
  kElevatorSimGravity = False
  kLiftMotorStdDevs = 0.01
