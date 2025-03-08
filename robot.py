from wpilib import Timer, XboxController
from wpinet import PortForwarder
from commands2 import TimedCommandRobot, SequentialCommandGroup, InstantCommand, WaitCommand
from commands2.button import Trigger
from robotcontainer import RobotContainer

class MyRobot(TimedCommandRobot):
  def robotInit(self):
    self.container = RobotContainer()
    self.swerve = self.container.swerve
    self.elevator = self.container.elevator
    self.poseEstimator = self.container.poseEstimator
    self.driverJoystick = self.container.driverJoystick

    self.timer = Timer()
    Trigger(lambda: self.timer.hasElapsed(110/7)).onTrue(
      SequentialCommandGroup(
        InstantCommand(
          lambda: self.driverJoystick.setRumble(XboxController.RumbleType.kBothRumble, 1)
        ),
        WaitCommand(5),
        InstantCommand(
          lambda: self.driverJoystick.setRumble(XboxController.RumbleType.kBothRumble, 0)
        )
      )
    )
    
    PortForwarder.getInstance().add(5800, "photonvision.local", 5800)

  def autonomousInit(self):
    self.autonomousCommand = self.container.getAutonomousCommand()
    if self.autonomousCommand:
      self.autonomousCommand.schedule()

  def teleopInit(self):
    self.swerve.zeroHeading(self.poseEstimator.getRotation2d())
    self.timer.restart()

  def teleopPeriodic(self):
    self.container.driverJoystick.setRumble(
      XboxController.RumbleType.kBothRumble,
      1 if 25 < Timer.getMatchTime() < 30 else 0
    )

  def disabledInit(self):
    self.container.stopAll()
    self.timer.stop()
