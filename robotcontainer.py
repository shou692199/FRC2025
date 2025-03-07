from wpilib import SmartDashboard, DriverStation
from wpimath.units import degreesToRadians
from commands import ClimbJoystick, DriveJoystick, GotoPreset, IntakeAlgae, IntakeCoral
from commands import ScoreCoral, ScoreCoralSlow, ShootAlgae
from commands2 import Command, InstantCommand, WaitCommand, ParallelCommandGroup, SequentialCommandGroup
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from pathplannerlib.path import PathPlannerPath, PathConstraints
from subsystems import Swerve, Elevator, Shooter, Pivot, Climber, PoseEstimator
from constants import AutoConstants, OIConstants, MotionPresets, OperationMode

class RobotContainer:
  def __init__(self):
    self.swerve = Swerve()
    self.elevator = Elevator()
    self.shooter = Shooter()
    self.pivot = Pivot()
    self.climber = Climber()
    self.poseEstimator = PoseEstimator(self.swerve)

    self.driverJoystick = CommandXboxController(OIConstants.kDriverControllerPort)
    self.operatorJoystick = CommandXboxController(OIConstants.kOperatorControllerPort)

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

    self.configureButtonBindings()
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

    self.climber.setDefaultCommand(
      ClimbJoystick(
        self.climber,
        lambda: self.driverJoystick.getRightTriggerAxis() - self.driverJoystick.getLeftTriggerAxis()
      )
    )

  def configureButtonBindings(self):
    self.driverJoystick.start().onTrue(
      InstantCommand(lambda: self.poseEstimator.setVisionEnabled(True))
    )
    self.driverJoystick.back().onTrue(
      InstantCommand(lambda: self.poseEstimator.setVisionEnabled(False))
    )
    self.driverJoystick.x().and_(self.driverJoystick.povDown()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef A")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povDown()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef B")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povDownRight()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef C")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povDownRight()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef D")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povUpRight()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef E")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povUpRight()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef F")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povUp()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef G")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povUp()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef H")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povUpLeft()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef I")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povUpLeft()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef J")
    )
    self.driverJoystick.x().and_(self.driverJoystick.povDownLeft()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef K")
    )
    self.driverJoystick.b().and_(self.driverJoystick.povDownLeft()).onTrue(
      self.getPathfindThenFollowPathCommand("Reef L")
    )
    self.driverJoystick.y().and_(self.driverJoystick.leftBumper()).onTrue(
      ParallelCommandGroup(
        self.getPathfindThenFollowPathCommand("Left Coral Station"),
        GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.CORAL_STATION)
      )
    )
    self.driverJoystick.y().and_(self.driverJoystick.rightBumper()).onTrue(
      ParallelCommandGroup(
        self.getPathfindThenFollowPathCommand("Right Coral Station"),
        GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.CORAL_STATION)
      )
    )

    self.operatorJoystick.start().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.CORAL_STATION)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.back().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.HOME)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povUp().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.SCORE_L1)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral(True)))
    self.operatorJoystick.povRight().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.SCORE_L2)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povDown().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.SCORE_L3)
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.leftBumper().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.REEF_L2)
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.rightBumper().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.REEF_L3)
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.leftTrigger().onTrue(
      InstantCommand(lambda: self.shooter.setCoralSensorEnabled(False))
    )
    self.operatorJoystick.rightTrigger().onTrue(
      InstantCommand(lambda: self.shooter.setCoralSensorEnabled(True))
    )
    self.operatorJoystick.leftStick().onTrue(InstantCommand(self.stopAll))
    self.operatorJoystick.x().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.PROCCESSOR)
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.y().onTrue(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.GROUND)
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.a().and_(
      lambda: self.operationMode == OperationMode.CORAL and self.isScoreCoralSlow).onTrue(
      ScoreCoralSlow(self.shooter, self.pivot)
    )
    self.operatorJoystick.a().and_(
      lambda: self.operationMode == OperationMode.CORAL and not self.isScoreCoralSlow).onTrue(
      ScoreCoral(self.shooter, self.pivot)
    )
    self.operatorJoystick.b().and_(
      lambda: self.operationMode == OperationMode.CORAL).whileTrue(
      IntakeCoral(self.shooter, self.pivot).until(self.shooter.isCoralFilled)
    )
    self.operatorJoystick.a().and_(lambda: self.operationMode == OperationMode.ALGAE).onTrue(
      ShootAlgae(self.shooter)
    )
    self.operatorJoystick.b().and_(lambda: self.operationMode == OperationMode.ALGAE).onTrue(
      IntakeAlgae(self.shooter)
    )

  def shouldFlipPath(self):
    alliance = DriverStation.getAlliance()
    return False if alliance is None else alliance == DriverStation.Alliance.kRed

  def setOperationMode(self, mode: OperationMode):
    self.operationMode = mode

  def setScoreCoral(self, shouldSlowdown = False):
    self.isScoreCoralSlow = shouldSlowdown
    self.setOperationMode(OperationMode.CORAL)

  def stopAll(self):
    self.swerve.stop()
    self.elevator.stop()
    self.shooter.stop()
    self.pivot.stop()
    self.climber.stop()

  def getAutonomousCommand(self) -> Command:
    selected = str(SmartDashboard.getString("Reef Algae Selector", "")).strip()
    if selected == "" : return None#or not self.poseEstimator.isVisionAvailable(): return None
    keyMapping = {"1": "AB", "2": "CD", "3": "EF", "4": "GH", "5": "IJ", "6": "KL"}
    autoCommand = SequentialCommandGroup(
      GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.SCORE_L1),
      self.getPathfindThenFollowPathCommand("Reef " + keyMapping.get(selected[0])),
      ScoreCoralSlow(self.shooter, self.pivot)
    )
    if len(selected) == 1:
      return autoCommand
    for s in selected[1:]:
      autoCommand = autoCommand.andThen(
        SequentialCommandGroup(
          ParallelCommandGroup(
            GotoPreset(
              self.elevator,
              self.shooter,
              self.pivot,
              MotionPresets.REEF_L2 if int(s) % 2 == 0 else MotionPresets.REEF_L3
            ).beforeStarting(WaitCommand(1)),
            self.getPathfindThenFollowPathCommand("Reef " + keyMapping.get(s))
          ),
          IntakeAlgae(self.shooter, True),
          ParallelCommandGroup(
            GotoPreset(self.elevator, self.shooter, self.pivot, MotionPresets.PROCCESSOR)
            .beforeStarting(WaitCommand(1)),
            self.getPathfindThenFollowPathCommand("Proccessor")
          ),
          ShootAlgae(self.shooter)
        )
      )
    return autoCommand

  def getPathfindThenFollowPathCommand(self, pathName: str):
    path = PathPlannerPath.fromPathFile(pathName)
    constraints = PathConstraints(
      3, 2,
      degreesToRadians(540), degreesToRadians(360)
    )
    return AutoBuilder.pathfindThenFollowPath(path, constraints)
