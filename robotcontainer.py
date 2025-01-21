from wpilib import SmartDashboard, DriverStation
from commands.defaultdrive import DefaultDrive
from commands2 import Command, InstantCommand
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from subsystems.swerve import Swerve
from constants import AutoConstants, OIConstants

class RobotContainer:
  def __init__(self):
    self.swerve = Swerve()
    self.driverJoystick = CommandXboxController(OIConstants.kDriverControllerPort)

    self.configureButtonBindings()

    if not AutoBuilder.isConfigured():
      AutoBuilder.configure(
        self.swerve.getPose,
        self.swerve.resetOdometry,
        self.swerve.getChassisSpeeds,
        lambda speed, _: self.swerve.driveRobotRelative(speed),
        PPHolonomicDriveController(
          PIDConstants(AutoConstants.kPTranslation),
          PIDConstants(AutoConstants.kPRotation)
        ),
        AutoConstants.kRobotConfig,
        self.shouldFlipPath,
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
    self.driverJoystick.y().onTrue(InstantCommand(self.swerve.zeroHeading))

  def shouldFlipPath(self):
    alliance = DriverStation.getAlliance()
    if alliance is None:
      return False
    return alliance == DriverStation.Alliance.kRed

  def getAutonomousCommand(self) -> Command:
    return self.autoChooser.getSelected()
