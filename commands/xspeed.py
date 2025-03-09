import commands2
from wpimath.kinematics import ChassisSpeeds
from subsystems import Swerve

class XSpeed(commands2.Command):
  def __init__(
      self,
      swerve: Swerve,
      xSpeed: float
  ):
    self.swerve = swerve
    self.xSpeed = xSpeed

    self.addRequirements(self.swerve)

  def initialize(self):
    self.swerve.driveRobotRelative(ChassisSpeeds(self.xSpeed, 0, 0))

  def end(self, interrupted):
    self.swerve.stop()

  def isFinished(self):
    return False
