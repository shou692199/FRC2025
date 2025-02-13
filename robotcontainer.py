from wpilib import SmartDashboard, DriverStation
from commands import ClimbCage, DriveJoystick, IntakeAlgae, IntakeCoral, ReleaseClimber
from commands import ScoreCoral, ScoreCoralSlow, SetElevatorHeight, SetShooterPitch, ShootAlgae
from commands.auto import ChaseTag
from commands2 import Command, InstantCommand
from commands2.button import CommandXboxController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
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
    self.configureButtonBindings()

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

    self.autoChooser = AutoBuilder.buildAutoChooser()
    SmartDashboard.putData("Auto Chooser", self.autoChooser)

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

  def configureButtonBindings(self):
    self.driverJoystick.start().onTrue(
      InstantCommand(lambda: self.poseEstimator.setVisionEnabled(True))
    )
    self.driverJoystick.back().onTrue(
      InstantCommand(lambda: self.poseEstimator.setVisionEnabled(False))
    )
    self.driverJoystick.x().onTrue(
      InstantCommand(lambda: self.swerve.zeroHeading(self.poseEstimator.getRotation2d())))
    self.driverJoystick.leftBumper().whileTrue(
      ChaseTag(VisionConstants.kMainCameraPhoton, self.swerve, self.poseEstimator.getPose)
    )
    self.driverJoystick.povUp().whileTrue(ClimbCage(self.climber))
    self.driverJoystick.povDown().onTrue(ReleaseClimber(self.climber))

    self.operatorJoystick.start().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.CORAL_STATION).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.CORAL_STATION)
      )
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.back().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.HOME).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.HOME)
      )
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povUp().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.SCORE_L1).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.SCORE_L1)
      )
    ).onTrue(InstantCommand(lambda: self.setScoreCoral(True)))
    self.operatorJoystick.povRight().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.SCORE_L2).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.SCORE_L2)
      )
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povDown().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.SCORE_L3).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.SCORE_L3)
      )
    ).onTrue(InstantCommand(lambda: self.setScoreCoral()))
    self.operatorJoystick.povLeft().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.SCORE_L4).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.SCORE_L4)
      )
    ).onTrue(InstantCommand(lambda: self.setScoreCoral(True)))
    self.operatorJoystick.leftBumper().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.REEF_L2).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.REEF_L2)
      )
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.rightBumper().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.REEF_L3).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.REEF_L3)
      )
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.x().onTrue(
      SetShooterPitch(self.shooter, MotionPresets.PROCCESSOR).andThen(
        SetElevatorHeight(self.elevator, MotionPresets.PROCCESSOR)
      )
    ).onTrue(InstantCommand(lambda: self.setOperationMode(OperationMode.ALGAE)))
    self.operatorJoystick.a().and_(
      lambda: self.operationMode == OperationMode.CORAL and self.isScoreCoralSlow).onTrue(
      ScoreCoralSlow(self.shooter)
    )
    self.operatorJoystick.a().and_(
      lambda: self.operationMode == OperationMode.CORAL and not self.isScoreCoralSlow).onTrue(
      ScoreCoral(self.shooter)
    )
    self.operatorJoystick.b().and_(
      lambda: self.operationMode == OperationMode.CORAL).onTrue(
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
    return self.autoChooser.getSelected()
