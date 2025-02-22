from wpilib import SmartDashboard, DriverStation
from wpimath.units import degreesToRadians
from commands import ClimbCage, DriveJoystick, GotoPreset, IntakeAlgae, IntakeCoral, ReleaseClimber
from commands import ScoreCoral, ScoreCoralSlow, SetElevatorHeight, SetShooterPitch, ShootAlgae
from commands2 import Command, InstantCommand, WaitCommand, RepeatCommand, ParallelCommandGroup, SequentialCommandGroup
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.path import PathPlannerPath, PathConstraints
from subsystems import Swerve, Elevator, Shooter, Climber, PoseEstimator
from constants import AutoConstants, VisionConstants, OIConstants, MotionPresets, OperationMode

class RobotContainer:
  def __init__(self):
    self.swerve = Swerve()
    self.elevator = Elevator()
    self.shooter = Shooter()
    self.climber = Climber()
    self.poseEstimator = PoseEstimator(
      VisionConstants.kMainCameraPhoton,
      VisionConstants.kMainCameraTransform,
      self.swerve
    )

    self.driverJoystick = CommandXboxController(OIConstants.kDriverControllerPort)
    self.operatorJoystick = CommandXboxController(OIConstants.kOperatorControllerPort)

    if not AutoBuilder.isConfigured():
      AutoBuilder.configure(
        self.poseEstimator.getPose,
        self.poseEstimator.resetPose,
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

    self.registerNamedCommands()
    self.configureButtonBindings()

    self.autoChooser = AutoBuilder.buildAutoChooser()
    SmartDashboard.putData("Auto Chooser", self.autoChooser)
    SmartDashboard.putString("Reef Algae Selector", "")

    self.operationMode = OperationMode.NONE
    self.isScoreCoralSlow = False

    self.swerve.setDefaultCommand(
      DriveJoystick(
        self.swerve,
        lambda: -self.driverJoystick.getLeftY(),
        lambda: -self.driverJoystick.getLeftX(),
        lambda: -self.driverJoystick.getRightX()
      )
    )

  def registerNamedCommands(self):
    NamedCommands.registerCommand(
      "Goto Preset SCORE_L1", GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L1)
    )
    NamedCommands.registerCommand(
      "Goto Preset REEF_L2", GotoPreset(self.elevator, self.shooter, MotionPresets.REEF_L2)
    )
    NamedCommands.registerCommand(
      "Goto Preset REEF_L3", GotoPreset(self.elevator, self.shooter, MotionPresets.REEF_L3)
    )
    NamedCommands.registerCommand(
      "Goto Preset PROCCESSOR", GotoPreset(self.elevator, self.shooter, MotionPresets.PROCCESSOR)
    )
    NamedCommands.registerCommand("Intake Algae", IntakeAlgae(self.shooter))
    NamedCommands.registerCommand("Intake Coral", IntakeCoral(self.shooter))
    NamedCommands.registerCommand("Score Coral", ScoreCoral(self.shooter))
    NamedCommands.registerCommand("Score Coral Slow", ScoreCoralSlow(self.shooter))
    NamedCommands.registerCommand("Shoot Algae", ShootAlgae(self.shooter))
    NamedCommands.registerCommand("Release Climber", ReleaseClimber(self.climber).raceWith(WaitCommand(1.5)))
    NamedCommands.registerCommand("Pathfind Reef AB", self.getPathfindThenFollowPathCommand("Reef AB"))
    NamedCommands.registerCommand("Pathfind Reef CD", self.getPathfindThenFollowPathCommand("Reef CD"))
    NamedCommands.registerCommand("Pathfind Reef EF", self.getPathfindThenFollowPathCommand("Reef EF"))
    NamedCommands.registerCommand("Pathfind Reef GH", self.getPathfindThenFollowPathCommand("Reef GH"))
    NamedCommands.registerCommand("Pathfind Reef IJ", self.getPathfindThenFollowPathCommand("Reef IJ"))
    NamedCommands.registerCommand("Pathfind Reef KL", self.getPathfindThenFollowPathCommand("Reef KL"))

  def configureButtonBindings(self):
    self.driverJoystick.start().onTrue(
      InstantCommand(lambda: self.poseEstimator.setVisionEnabled(True))
    )
    self.driverJoystick.back().onTrue(
      InstantCommand(lambda: self.poseEstimator.setVisionEnabled(False))
    )
    self.driverJoystick.leftBumper().onTrue(
      self.getPathfindThenFollowPathCommand("Left Coral Station")
      .alongWith(GotoPreset(self.elevator, self.shooter, MotionPresets.CORAL_STATION))
    )
    self.driverJoystick.rightBumper().onTrue(
      self.getPathfindThenFollowPathCommand("Right Coral Station")
      .alongWith(GotoPreset(self.elevator, self.shooter, MotionPresets.CORAL_STATION))
    )
    self.driverJoystick.povUp().whileTrue(ClimbCage(self.climber))
    self.driverJoystick.povDown().whileTrue(ReleaseClimber(self.climber))
    self.driverJoystick.x().onTrue(
      InstantCommand(lambda: self.swerve.zeroHeading(self.poseEstimator.getRotation2d())))

    self.operatorJoystick.start().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.CORAL_STATION)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.back().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.HOME)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povUp().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L1)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral(True)))
    self.operatorJoystick.povRight().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L2)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povDown().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L3)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povLeft().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L4)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral(True)))
    self.operatorJoystick.leftBumper().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.REEF_L2)
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.rightBumper().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.REEF_L3)
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.leftStick().onTrue(InstantCommand(self.stopAll))
    self.operatorJoystick.x().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.PROCCESSOR)
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.y().onTrue(
      GotoPreset(self.elevator, self.shooter, MotionPresets.SCORE_L4)
      .andThen(SetShooterPitch(self.shooter, 45))
      .andThen(ShootAlgae(self.shooter))
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.NET)))
    self.operatorJoystick.a().and_(
      lambda: self.operationMode == OperationMode.CORAL and self.isScoreCoralSlow).onTrue(
      ScoreCoralSlow(self.shooter)
    )
    self.operatorJoystick.a().and_(
      lambda: self.operationMode == OperationMode.CORAL and not self.isScoreCoralSlow).onTrue(
      ScoreCoral(self.shooter)
    )
    self.operatorJoystick.b().and_(
      lambda: self.operationMode == OperationMode.CORAL).whileTrue(
      IntakeCoral(self.shooter)
    )
    self.operatorJoystick.a().and_(lambda: self.operationMode == OperationMode.ALGAE).onTrue(
      ShootAlgae(self.shooter)
    )
    self.operatorJoystick.b().and_(lambda: self.operationMode == OperationMode.ALGAE).onTrue(
      IntakeAlgae(self.shooter)
    )

  def shouldFlipPath(self):
    alliance = DriverStation.getAlliance()
    if alliance is None:
      return False
    return alliance == DriverStation.Alliance.kRed

  def setOperationMode(self, mode: OperationMode):
    self.operationMode = mode

  def setScoreCoral(self, shouldSlowdown = False):
    self.isScoreCoralSlow = shouldSlowdown
    self.setOperationMode(OperationMode.CORAL)

  def stopAll(self):
    self.swerve.stop()
    self.elevator.stop()
    self.shooter.stop()
    self.climber.stop()

  def getAutonomousCommand(self) -> Command:
    return None
    autoCommand = self.autoChooser.getSelected()
    selstr = str(SmartDashboard.getString("Reef Algae Selector", "")).strip()
    keyMapping = {"1": "AB", "2": "CD", "3": "EF", "4": "GH", "5": "IJ", "6": "KL"}
    if selstr == "":
      return autoCommand
    for s in selstr:
      autoCommand = autoCommand.andThen(
        ParallelCommandGroup(
          GotoPreset(
            self.elevator,
            self.shooter,
            MotionPresets.REEF_L2 if int(s) % 2 == 0 else MotionPresets.REEF_L3
          ),
          self.getPathfindThenFollowPathCommand("Reef " + keyMapping.get(s))
        ).andThen(IntakeAlgae(self.shooter, True)).andThen(
          ParallelCommandGroup(
            GotoPreset(self.elevator, self.shooter, MotionPresets.PROCCESSOR),
            self.getPathfindThenFollowPathCommand("Proccessor")
          ).andThen(ShootAlgae(self.shooter))
        )
      )
    return autoCommand

  def getPathfindThenFollowPathCommand(self, pathName: str):
    path = PathPlannerPath.fromPathFile(pathName)
    constraints = PathConstraints(
      3.0, 3.0,
      degreesToRadians(540), degreesToRadians(360)
    )
    pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
      path,
      constraints
    )
    return pathfindingCommand
