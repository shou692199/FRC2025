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
  kDriveMotorNominalVoltage = 12
  kPDriveMotor = 0.95
  kSDriveMotor = 0.06
  kVDriveMotor = kDriveMotorNominalVoltage * 60 / kDriveEncoderRot2Meter / kDriveMotorMaxRPM
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
  kPHeading = 1.4

  kPhysicalMaxSpeedMetersPerSecond = 5
  kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * math.tau
  kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2
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
  kMotorId = 14
  kMotorSubId = 15

  kChainPitchMeters = inchesToMeters(0.25)
  kSprocketTeeth = 26
  kMotorGearRatio = 1 / 20
  kEncoderRot2Meter = kMotorGearRatio * kSprocketTeeth * kChainPitchMeters * 2

  kSmartCurrentLimit = 30
  kForwardLimitMeters = 1.1
  kReverseLimitMeters = 0
  kMaxOutput = 0.35
  kPMotorPosition = 20

class PivotConstants:
  kPitchMotorId = 20
  kRollerMotorId = 21

  kPitchMotorGearRatio = 1 / 20
  kPitchSprocketRatio = 18 / 56
  kPitchEncoderRot2Deg = kPitchMotorGearRatio * kPitchSprocketRatio * 360
  kPitchAbsoluteEncoderOffset = 136

  kSmartCurrentLimit = 60
  kPitchMaxOutput = 0.3
  kPPitchMotor = 0.005

class ShooterConstants:
  kPitchMotorId = 16
  kRollerMotorId = 17

  kPitchMotorGearRatio = 1 / 20
  kPitchSprocketRatio = 14 / 50
  kPitchEncoderRot2Deg = kPitchMotorGearRatio * kPitchSprocketRatio * 360
  kPitchAbsoluteEncoderOffset = 0.9

  kSmartCurrentLimit = 60
  kPitchMaxOutput = 0.7
  kPPitchMotor = 0.02

class ClimberConstants:
  kMotorId = 18
  kMotorSubId = 19
  kSmartCurrentLimit = 40
  kAbsoluteEncoderOffset = 0.69

class MotionPresets(Enum):
  CORAL_STATION = (0, 182.65, 64)
  PROCCESSOR = (0.1, 165, 0)
  SCORE_L1 = (0.25, 165, 60)
  SCORE_L2 = (0.52, 165, 50)
  SCORE_L3 = (0.88, 165, 47)
  REEF_L2 = (0.55, 165, 0)
  REEF_L3 = (0.90, 165, 0)
  HOME = (0, 165, 0)
  GROUND = (0, 90, 65)

class OperationMode(Enum):
  NONE = 0
  CORAL = 1
  ALGAE = 2

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

  kTag2GoalTransform = Transform3d(1.5, 0, 0, Rotation3d(0, 0, 0))

class OIConstants:
  kDriverControllerPort = 0
  kOperatorControllerPort = 1
  kDeadband = 0.06

class VisionConstants:
  kHighCameraPhoton = PhotonCamera("High Camera")
  kHighCameraTransform = Transform3d(
    0.230, 0.295, 1.015, Rotation3d()
  )
  kLowCameraPhoton = PhotonCamera("Low Camera")
  kLowCameraTransform = Transform3d(
    0.22, -0.31, 0.325, Rotation3d()
  )
  kStateStdDevs = (0.05, 0.05, degreesToRadians(5))
  kVisionMesurementStdDevs = (0.5, 0.5, degreesToRadians(30))

class PhysicsConstants:
  kDriveMotorMOI = 0.0007
  kSteerMotorMOI = 0.0001
  kGyroSimDevice = "navX-Sensor[4]"
  kElevatorMassKilograms = 4
  kElevatorSimGravity = False
  kLiftMotorStdDevs = 0.01
