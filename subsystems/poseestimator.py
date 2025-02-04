import commands2
from wpilib import SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Transform3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from ntcore import NetworkTableInstance
from subsystems import Swerve
from constants import VisionConstants, DriveConstants

class PoseEstimator(commands2.Subsystem):
  def __init__(
    self,
    camera: PhotonCamera,
    cameraTransform: Transform3d,
    swerve: Swerve
  ):
    self.camera = camera
    self.cameraTransform = cameraTransform
    self.swerve = swerve

    self.photonPoseEstimator = PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape),
      PoseStrategy.LOWEST_AMBIGUITY,
      camera,
      cameraTransform
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

  def resetOdometry(self, pose: Pose2d):
    self.poseEstimator.resetPose(pose)

  def getEstimatedPose(self):
    return self.poseEstimator.getEstimatedPosition()

  def periodic(self):
    estimatedPose = self.photonPoseEstimator.update()
    if estimatedPose:
      self.poseEstimator.addVisionMeasurement(
        estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds)
    self.poseEstimator.update(self.swerve.getRotation2d(), self.swerve.getModulePositions())
    self.posePublisher.set(self.getEstimatedPose())