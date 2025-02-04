from wpilib import SmartDashboard, DriverStation
from commands.manual import DefaultDrive, ElevatorCtrl
from commands2 import Command, InstantCommand
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from photonlibpy.photonCamera import PhotonCamera
from subsystems import Swerve, Elevator, PoseEstimator
from constants import AutoConstants, VisionConstants, OIConstants, ReefLayers

class RobotContainer:
  def __init__(self):
    self.swerve = Swerve()
    self.elevator = Elevator()
    self.poseEstimator = PoseEstimator(
      PhotonCamera(VisionConstants.kMainCameraName),
      VisionConstants.kMainCameraTransform,
      self.swerve
    )

    self.driverJoystick = CommandXboxController(OIConstants.kDriverControllerPort)
    self.configureButtonBindings()

    if not AutoBuilder.isConfigured():
      AutoBuilder.configure(
        self.poseEstimator.getEstimatedPose,
        self.poseEstimator.resetOdometry,
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
    self.driverJoystick.start().onTrue(InstantCommand(self.swerve.zeroHeading))
    self.driverJoystick.y().whileTrue(ElevatorCtrl(self.elevator, True))
    self.driverJoystick.a().whileTrue(ElevatorCtrl(self.elevator, False))
    self.driverJoystick.povUp().onTrue(
      InstantCommand(lambda: self.elevator.setDesiredHeight(ReefLayers.L1))
    )
    self.driverJoystick.povRight().onTrue(
      InstantCommand(lambda: self.elevator.setDesiredHeight(ReefLayers.L2))
    )
    self.driverJoystick.povDown().onTrue(
      InstantCommand(lambda: self.elevator.setDesiredHeight(ReefLayers.L3))
    )
    self.driverJoystick.povLeft().onTrue(
      InstantCommand(lambda: self.elevator.setDesiredHeight(ReefLayers.L4))
    )

  def shouldFlipPath(self):
    alliance = DriverStation.getAlliance()
    if alliance is None:
      return False
    return alliance == DriverStation.Alliance.kRed

  def getAutonomousCommand(self) -> Command:
    return self.autoChooser.getSelected()
