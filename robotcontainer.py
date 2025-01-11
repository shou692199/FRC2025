from wpilib import SmartDashboard
from commands.defaultdrive import DefaultDrive
from commands2 import Command, InstantCommand
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder
from subsystems.swervesubsystem import SwerveSubsystem

class RobotContainer:
  def __init__(self):
    self.swerve = SwerveSubsystem()
    self.driverJoystick = CommandXboxController(0)

    self.configureButtonBindings()

    self.autoChooser = AutoBuilder.buildAutoChooser()
    SmartDashboard.putData("Auto Chooser", self.autoChooser)

    self.swerve.setDefaultCommand(
      DefaultDrive(
        self.swerve,
        lambda: -self.driverJoystick.getLeftY(),
        lambda: -self.driverJoystick.getLeftX(),
        lambda: -self.driverJoystick.getRightX()
      )
    )

  def configureButtonBindings(self):
    self.driverJoystick.x().onTrue(InstantCommand(self.swerve.zeroHeading))
    pass

  def getAutonomousCommand(self) -> Command:
    return self.autoChooser.getSelected()
