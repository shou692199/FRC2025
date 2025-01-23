import commands2
from robotcontainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
  def robotInit(self):
    self.container = RobotContainer()
    self.swerve = self.container.swerve
    self.elevator = self.container.elevator

  def autonomousInit(self):
    self.autonomousCommand = self.container.getAutonomousCommand()
    if self.autonomousCommand:
      self.autonomousCommand.schedule()