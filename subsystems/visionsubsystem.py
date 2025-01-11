import commands2
from photonlibpy.photonCamera import PhotonCamera
from robotpy_apriltag import AprilTagField
from constants import VisionConstants

class VisionSubsystem(commands2.Subsystem):
  def __init__(self):
    self.mainCamera = PhotonCamera(VisionConstants.kMainCameraName)
    self.aprilTagFieldLayout = AprilTagField.kDefaultField

  def periodic(self):
    pass