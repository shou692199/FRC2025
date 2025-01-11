import threading
import time
import math
import commands2
from typing import Iterable
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from wpilib import DriverStation
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry, ChassisSpeeds
from navx import AHRS
from swervemodule import SwerveModule
from constants import DriveConstants

class SwerveSubsystem(commands2.Subsystem):
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
    self.odometer = SwerveDrive4Odometry(
      DriveConstants.kDriveKinematics, Rotation2d(0), self.getModulePositions()
    )

    AutoBuilder.configure(
      self.getPose,
      self.resetOdometry,
      self.getChassisSpeeds,
      lambda speed, feed: self.driveRobotRelative(speed),
      PPHolonomicDriveController(PIDConstants(5), PIDConstants(5)),
      DriveConstants.kRobotConfig,
      self.shouldFlipPath,
      self
    )
    
    threading.Thread(target=self.zeroHeading, args=(1,)).start()

  def zeroHeading(self, delay: int = 0):
    time.sleep(delay)
    self.gyro.reset()
  
  def resetOdometry(self, pose: Pose2d):
    self.odometer.resetPosition(self.getRotation2d(), self.getModulePositions(), pose)

  def getHeading(self) -> float:
    if self.gyro.isCalibrating():
      return 0
    return -math.remainder(self.gyro.getAngle(), 360)
  
  def getRotation2d(self):
    return Rotation2d.fromDegrees(self.getHeading())
  
  def getPose(self):
    return self.odometer.getPose()
  
  def getModulePositions(self):
    return tuple(m.getModulePosition() for m in self.modules)
  
  def getModuleStates(self):
    return tuple(m.getModuleState() for m in self.modules)
  
  def setModuleStates(self, desiredState: Iterable[SwerveModuleState]):
    desiredState = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      desiredState, DriveConstants.kPhysicalMaxSpeedMetersPerSecond
    )
    for i in range(4):
      self.modules[i].setDesiredState(desiredState[i], str(i))

  def getChassisSpeeds(self):
    return DriveConstants.kDriveKinematics.toChassisSpeeds(self.getModuleStates())

  def driveRobotRelative(self, chassisSpeeds: ChassisSpeeds):
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    self.setModuleStates(moduleStates)

  def stopModules(self):
    for m in self.modules:
      m.stop()

  def shouldFlipPath(self):
    alliance = DriverStation.getAlliance()
    if alliance is None:
      return False
    return alliance == DriverStation.Alliance.kRed

  def periodic(self):
    self.odometer.update(self.getRotation2d(), self.getModulePositions())
