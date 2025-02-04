import math
import commands2
from typing import Iterable
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry, ChassisSpeeds
from ntcore import NetworkTableInstance
from navx import AHRS
from swervemodule import SwerveModule
from constants import DriveConstants

class Swerve(commands2.Subsystem):
  def __init__(self):
    self.frontLeft = SwerveModule(
      DriveConstants.kFrontLeftDriveMotorId,
      DriveConstants.kFrontLeftSteerMotorId,
      DriveConstants.kFrontLeftShaftEncoderId,
      DriveConstants.kFrontLeftShaftEncoderOffset
    )

    self.frontRight = SwerveModule(
      DriveConstants.kFrontRightDriveMotorId,
      DriveConstants.kFrontRightSteerMotorId,
      DriveConstants.kFrontRightShaftEncoderId,
      DriveConstants.kFrontRightShaftEncoderOffset
    )

    self.backLeft = SwerveModule(
      DriveConstants.kBackLeftDriveMotorId,
      DriveConstants.kBackLeftSteerMotorId,
      DriveConstants.kBackLeftShaftEncoderId,
      DriveConstants.kBackLeftShaftEncoderOffset
    )

    self.backRight = SwerveModule(
      DriveConstants.kBackRightDriveMotorId,
      DriveConstants.kBackRightSteerMotorId,
      DriveConstants.kBackRightShaftEncoderId,
      DriveConstants.kBackRightShaftEncoderOffset
    )

    self.modules = [self.frontLeft, self.frontRight, self.backLeft, self.backRight]
    self.gyro = AHRS(AHRS.NavXComType.kMXP_SPI)
    self.gyro.enableBoardlevelYawReset(True)

    self.desiredHeading = float(0)
    self.headingPIDController = PIDController(DriveConstants.kPHeading, 0, 0)
    self.headingPIDController.enableContinuousInput(-math.pi, math.pi)
    
    self.zeroHeading()

    nt = NetworkTableInstance.getDefault()
    self.statePublisher = nt.getStructArrayTopic("/SwerveStates", SwerveModuleState).publish()

  def zeroHeading(self):
    self.gyro.reset()
    self.desiredHeading = self.getRotation2d().radians()

  def getHeading(self):
    return -self.gyro.getYaw()
  
  def getRotation2d(self):
    return Rotation2d.fromDegrees(self.getHeading())
  
  def getModulePositions(self):
    return tuple(m.getModulePosition() for m in self.modules)
  
  def getModuleStates(self):
    return tuple(m.getModuleState() for m in self.modules)
  
  def setModuleStates(self, desiredState: Iterable[SwerveModuleState]):
    desiredState = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      desiredState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond
    )
    for i in range(4):
      self.modules[i].setDesiredState(desiredState[i])

  def getChassisSpeeds(self):
    return DriveConstants.kDriveKinematics.toChassisSpeeds(self.getModuleStates())

  def driveRobotRelative(self, chassisSpeeds: ChassisSpeeds):
    differ = lambda a, b: abs((a - b + math.pi) % math.tau - math.pi)
    currentHeading = self.getRotation2d().radians()
    if (abs(chassisSpeeds.omega) > DriveConstants.kDeadband
        or differ(self.desiredHeading, currentHeading) > math.pi / 4
    ):
      self.desiredHeading = currentHeading
    elif (chassisSpeeds.vx**2 + chassisSpeeds.vy**2)**0.5 > DriveConstants.kDeadband:
      chassisSpeeds.omega = self.headingPIDController.calculate(currentHeading, self.desiredHeading)

    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    self.setModuleStates(moduleStates)

  def stopModules(self):
    for m in self.modules:
      m.stop()

  def periodic(self):
    self.statePublisher.set(list(self.getModuleStates()))
    SmartDashboard.putNumber("Heading", self.getRotation2d().degrees())