from wpilib import SmartDashboard
from commands.defaultdrive import DefaultDrive
from commands2 import Command, InstantCommand
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from subsystems.swervesubsystem import SwerveSubsystem
from constants import DriveConstants

class RobotContainer:
  def __init__(self):
    self.swerve = SwerveSubsystem()
    self.driverJoystick = CommandXboxController(0)

    self.configureButtonBindings()

    if not AutoBuilder.isConfigured():
      AutoBuilder.configure(
        self.swerve.getPose,
        self.swerve.resetOdometry,
        self.swerve.getChassisSpeeds,
        lambda speed, feed: self.swerve.driveRobotRelative(speed),
        PPHolonomicDriveController(PIDConstants(5), PIDConstants(5)),
        DriveConstants.kRobotConfig,
        self.swerve.shouldFlipPath,
        self.swerve
      )

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

  def getAutonomousCommand(self) -> Command:
    return self.autoChooser.getSelected()
