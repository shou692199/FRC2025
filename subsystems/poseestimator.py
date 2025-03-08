import commands2
from wpilib import SmartDashboard, Timer
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from ntcore import NetworkTableInstance
from subsystems import Swerve
from constants import VisionConstants, DriveConstants

class PoseEstimator(commands2.Subsystem):
  def __init__(
    self,
    swerve: Swerve
  ):
    self.swerve = swerve

    self.highPhotonPoseEstimator = PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded),
      PoseStrategy.LOWEST_AMBIGUITY,
      VisionConstants.kHighCameraPhoton,
      VisionConstants.kHighCameraTransform
    )

    self.lowPhotonPoseEstimator = PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded),
      PoseStrategy.LOWEST_AMBIGUITY,
      VisionConstants.kLowCameraPhoton,
      VisionConstants.kLowCameraTransform
    )

    self.poseEstimator = SwerveDrive4PoseEstimator(
      DriveConstants.kDriveKinematics,
      swerve.getRotation2d(),
      swerve.getModulePositions(),
      Pose2d(),
      VisionConstants.kStateStdDevs,
      VisionConstants.kVisionMesurementStdDevs
    )

    nt = NetworkTableInstance.getDefault()
    self.posePublisher = nt.getStructTopic("/SwervePose", Pose2d).publish()
    self.visionEnabled = True
    self.visionTimestamp = float(-1)

  def resetPose(self, pose: Pose2d):
    self.poseEstimator.resetPose(pose)

  def getPose(self):
    return self.poseEstimator.getEstimatedPosition()
  
  def getRotation2d(self):
    return self.getPose().rotation()

  def isVisionAvailable(self):
    return (
      self.visionEnabled
      and VisionConstants.kHighCameraPhoton.isConnected()
      and VisionConstants.kLowCameraPhoton.isConnected()
      and Timer.getTimestamp() - self.visionTimestamp < 5
    )

  def setVisionEnabled(self, enabled: bool):
    self.visionEnabled = enabled

  def addVisionMeasurement(self, estimatedPose: EstimatedRobotPose | None):
    if estimatedPose and self.visionEnabled:
      singleTarget = estimatedPose.targetsUsed[0]
      cameraToTarget = singleTarget.getBestCameraToTarget()
      distance = (cameraToTarget.x**2 + cameraToTarget.y**2)**0.5
      chassisSpeeds = self.swerve.getChassisSpeeds()
      absSpeed = abs((chassisSpeeds.vx**2 + chassisSpeeds.vy**2)**0.5)
      if singleTarget.getPoseAmbiguity() <= 0.2 and distance <= 3.5 and absSpeed <= 0.3:
        self.poseEstimator.addVisionMeasurement(
          estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds)
        self.visionTimestamp = Timer.getTimestamp()

  def periodic(self):
    self.addVisionMeasurement(self.highPhotonPoseEstimator.update())
    self.addVisionMeasurement(self.lowPhotonPoseEstimator.update())
    self.poseEstimator.update(self.swerve.getRawRotation2d(), self.swerve.getModulePositions())
    self.posePublisher.set(self.getPose())
    SmartDashboard.putBoolean("Vision Enabled", self.visionEnabled)
    SmartDashboard.putBoolean("Vision Available", self.isVisionAvailable())