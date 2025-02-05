import math
import commands2
from typing import Callable
from wpimath.controller import ProfiledPIDController, ProfiledPIDControllerRadians
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Pose3d
from photonlibpy.photonCamera import PhotonCamera
from subsystems import Swerve
from constants import ChaseTagConstants, VisionConstants

class ChaseTag(commands2.Command):
  def __init__(
      self,
      camera: PhotonCamera,
      swerve: Swerve, 
      getPose: Callable[[], Pose2d]
  ):
    self.camera = camera
    self.swerve = swerve
    self.getPose = getPose

    self.xController = ProfiledPIDController(
      ChaseTagConstants.kPXController,
      0,
      0,
      ChaseTagConstants.kXControllerConstraints
    )
    self.yController = ProfiledPIDController(
      ChaseTagConstants.kPYController,
      0,
      0,
      ChaseTagConstants.kYControllerConstraints
    )
    self.oController = ProfiledPIDControllerRadians(
      ChaseTagConstants.kPOController,
      0,
      0,
      ChaseTagConstants.kOControllerConstraints
    )

    self.xController.setTolerance(ChaseTagConstants.kXToleranceMeters)
    self.yController.setTolerance(ChaseTagConstants.kYToleranceMeters)
    self.oController.setTolerance(ChaseTagConstants.kOToleranceRadians)
    self.oController.enableContinuousInput(-math.pi, math.pi)

    self.addRequirements(self.swerve)

  def initialize(self):
    self.lastTarget = None
    robotPose = self.getPose()
    self.xController.reset(robotPose.X())
    self.yController.reset(robotPose.Y())
    self.oController.reset(robotPose.rotation().radians())

  def execute(self):
    target = None
    robotPose = Pose3d(self.getPose())
    cameraResult = self.camera.getLatestResult()
    if cameraResult.hasTargets():
      for t in cameraResult.getTargets():
        if t.getFiducialId() == 3 and t.getPoseAmbiguity() < 0.5:
          target = t
          break

    if target:
      self.lastTarget = target
      cameraPose = robotPose.transformBy(VisionConstants.kMainCameraTransform)
      targetPose = cameraPose.transformBy(target.getBestCameraToTarget())
      goalPose = targetPose.transformBy(ChaseTagConstants.kTag2GoalTransform).toPose2d()

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