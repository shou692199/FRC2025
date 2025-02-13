import commands2
from robotcontainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
  def robotInit(self):
    self.container = RobotContainer()
    self.swerve = self.container.swerve
    self.elevator = self.container.elevator
    self.poseEstimator = self.container.poseEstimator

    self.mechInitStatus = False

  def autonomousInit(self):
    self.autonomousCommand = self.container.getAutonomousCommand()
    if self.autonomousCommand:
      self.autonomousCommand.schedule()

  def teleopInit(self):
    self.swerve.zeroHeading(self.poseEstimator.getRotation2d())

  def disabledInit(self):
    self.container.stopAll()
