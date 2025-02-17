import math
from commands2 import Subsystem, Command, InstantCommand
from wpilib import SmartDashboard
from rev import SparkMax, SparkMaxConfig, LimitSwitchConfig, ClosedLoopConfig
from constants import ShooterConstants

class Shooter(Subsystem):
  def __init__(self):
    self.pitchMotor = SparkMax(
      ShooterConstants.kPitchMotorId, SparkMax.MotorType.kBrushless
    )
    self.rollerMotor = SparkMax(
      ShooterConstants.kRollerMotorId, SparkMax.MotorType.kBrushless
    )

    self.pitchRelativeEncoder = self.pitchMotor.getEncoder()
    self.pitchAbsoluteEncoder = self.pitchMotor.getAbsoluteEncoder()
    self.pitchClosedLoopController = self.pitchMotor.getClosedLoopController()

    self.rollerEncoder = self.rollerMotor.getEncoder()
    self.rollerClosedLoopController = self.rollerMotor.getClosedLoopController()

    self.configurePitchParam()
    self.configureRollerParam()

    self.resetEncoders()

    self.pitchForwardLimit = ShooterConstants.kPitchForwardLimitDeg
    self.pitchReverseLimit = ShooterConstants.kPitchReverseLimitDeg
    self.desiredPitch = float(0)

  def configurePitchParam(self):
    cfg_pitch = SparkMaxConfig()
    cfg_pitch.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    cfg_pitch.inverted(True)

    cfg_pitch.encoder.positionConversionFactor(ShooterConstants.kPitchEncoderRot2Deg)
    cfg_pitch.encoder.velocityConversionFactor(ShooterConstants.kPitchEncoderRot2Deg / 60)

    cfg_pitch.absoluteEncoder.zeroCentered(True)
    cfg_pitch.absoluteEncoder.zeroOffset(ShooterConstants.kPitchAbsoluteEncoderOffset)
    cfg_pitch.absoluteEncoder.positionConversionFactor(360)
    cfg_pitch.absoluteEncoder.velocityConversionFactor(6)

    cfg_pitch.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
    cfg_pitch.limitSwitch.forwardLimitSwitchEnabled(True)
    cfg_pitch.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
    cfg_pitch.limitSwitch.reverseLimitSwitchEnabled(True)

    cfg_pitch.softLimit.forwardSoftLimit(ShooterConstants.kPitchForwardLimitDeg)
    cfg_pitch.softLimit.forwardSoftLimitEnabled(True)
    cfg_pitch.softLimit.reverseSoftLimit(ShooterConstants.kPitchReverseLimitDeg)
    cfg_pitch.softLimit.reverseSoftLimitEnabled(True)

    cfg_pitch.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    cfg_pitch.closedLoop.P(ShooterConstants.kPPitchMotor)
    cfg_pitch.closedLoop.outputRange(
      -ShooterConstants.kPitchMaxOutput, ShooterConstants.kPitchMaxOutput
    )

    self.pitchMotor.configure(
      cfg_pitch,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

  def configureRollerParam(self):
    cfg_roller = SparkMaxConfig()
    cfg_roller.smartCurrentLimit(ShooterConstants.kSmartCurrentLimit)

    cfg_roller.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    cfg_roller.closedLoop.P(0.05)

    self.rollerMotor.configure(
      cfg_roller,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

  def resetEncoders(self):
    self.pitchRelativeEncoder.setPosition(self.getPitch())

  def getPitch(self):
    return self.pitchAbsoluteEncoder.getPosition()
  
  def setGoalPitch(self, pitch: float):
    self.desiredPitch = pitch
    self.pitchClosedLoopController.setReference(pitch, SparkMax.ControlType.kPosition)

  def atGoalPitch(self):
    return math.isclose(self.getPitch(), self.desiredPitch, abs_tol=5)

  def stopPitch(self):
    self.pitchMotor.stopMotor()

  def intakeCoralCommand(self) -> Command:
    return InstantCommand(lambda: self.rollerMotor.set(0.7))

  def outtakeCoralCommand(self, slowdown = False) -> Command:
    return InstantCommand(
      lambda: self.rollerMotor.set(-0.3 if slowdown else -0.8)
    )

  def intakeAlgaeCommand(self) -> Command:
    return InstantCommand(lambda: self.rollerMotor.set(-0.7))

  def outtakeAlgaeCommand(self) -> Command:
    return InstantCommand(lambda: self.rollerMotor.set(1))

  def holdAlgaeCommand(self) -> Command:
    return InstantCommand(lambda: self.rollerMotor.set(-0.2))

  def stopRoller(self):
    self.rollerMotor.stopMotor()

  def stop(self):
    self.stopPitch()
    self.stopRoller()

  def periodic(self):
    SmartDashboard.putNumber("Shooter Pitch", self.getPitch())
