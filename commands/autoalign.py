import math
import commands2
from typing import Callable
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Pose3d
from subsystems import Swerve
from constants import AutoAlignConstants, VisionConstants

class AutoAlign(commands2.Command):
  def __init__(
      self,
      swerve: Swerve, 
      getPose: Callable[[], Pose2d],
      shouldFlipPath: Callable[[], bool],
      isLeftReef: bool
  ):
    self.swerve = swerve
    self.getPose = getPose
    self.shouldFlipPath = shouldFlipPath
    self.isLeftReef = isLeftReef

    self.xController = ProfiledPIDController(
      AutoAlignConstants.kPXController,
      0,
      0,
      AutoAlignConstants.kXControllerConstraints
    )
    self.yController = ProfiledPIDController(
      AutoAlignConstants.kPYController,
      0,
      0,
      AutoAlignConstants.kYControllerConstraints
    )
    self.oController = ProfiledPIDControllerRadians(
      AutoAlignConstants.kPOController,
      0,
      0,
      AutoAlignConstants.kOControllerConstraints
    )

    self.xController.setTolerance(AutoAlignConstants.kXToleranceMeters)
    self.yController.setTolerance(AutoAlignConstants.kYToleranceMeters)
    self.oController.setTolerance(AutoAlignConstants.kOToleranceRadians)
    self.oController.enableContinuousInput(-math.pi, math.pi)

    self.addRequirements(self.swerve)

  def initialize(self):
    self.lastTarget = None
    self.reefIdMin = 6 if self.shouldFlipPath() else 17
    robotPose = self.getPose()
    self.xController.reset(robotPose.X())
    self.yController.reset(robotPose.Y())
    self.oController.reset(robotPose.rotation().radians())

  def execute(self):
    target = None
    robotPose = Pose3d(self.getPose())
    cameraResult = VisionConstants.kHighCameraPhoton.getLatestResult()
    if cameraResult.hasTargets():
      for t in cameraResult.getTargets():
        if self.reefIdMin <= t.getFiducialId() <= self.reefIdMin+5 and t.getPoseAmbiguity() < 0.1:
          target = t
          break

    if target:
      self.lastTarget = target
      cameraPose = robotPose.transformBy(VisionConstants.kHighCameraTransform)
      targetPose = cameraPose.transformBy(target.getBestCameraToTarget())
      goalPose = targetPose.transformBy(
        AutoAlignConstants.kLeftReefTransform if self.isLeftReef else AutoAlignConstants.kRightReefTransform
      ).toPose2d()

      self.xController.setGoal(goalPose.X())
      self.yController.setGoal(goalPose.Y())
      self.oController.setGoal(goalPose.rotation().radians())

    if self.lastTarget:
      xSpeed = self.xController.calculate(robotPose.X())
      ySpeed = self.yController.calculate(robotPose.Y())
      oSpeed = self.oController.calculate(robotPose.rotation().toRotation2d().radians())

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, oSpeed, robotPose.rotation().toRotation2d()
      )
      self.swerve.driveRobotRelative(chassisSpeeds)
    else:
      self.swerve.stopModules()
    
  def end(self, interrupted):
    self.swerve.stopModules()