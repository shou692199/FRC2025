import math
from commands2 import Subsystem
from wpilib import SmartDashboard, DutyCycleEncoder
from rev import SparkMax, SparkMaxConfig, ClosedLoopConfig
from constants import PivotConstants

class Pivot(Subsystem):
  def __init__(self):
    self.pitchMotor = SparkMax(
      PivotConstants.kPitchMotorId, SparkMax.MotorType.kBrushless
    )

    self.pitchRelativeEncoder = self.pitchMotor.getEncoder()
    self.pitchAbsoluteEncoder = DutyCycleEncoder(10, 360, PivotConstants.kPitchAbsoluteEncoderOffset)
    self.pitchClosedLoopController = self.pitchMotor.getClosedLoopController()

    self.configurePitchParam()
    self.configureRollerParam()
    self.resetEncoders()

    self.desiredPitch = float(0)

  def configurePitchParam(self):
    cfg_pitch = SparkMaxConfig()
    cfg_pitch.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

    cfg_pitch.encoder.positionConversionFactor(PivotConstants.kPitchEncoderRot2Deg)
    cfg_pitch.encoder.velocityConversionFactor(PivotConstants.kPitchEncoderRot2Deg / 60)

    cfg_pitch.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    cfg_pitch.closedLoop.P(PivotConstants.kPPitchMotor)
    cfg_pitch.closedLoop.outputRange(
      -PivotConstants.kPitchMaxOutput, PivotConstants.kPitchMaxOutput
    )

    self.pitchMotor.configure(
      cfg_pitch,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

  def configureRollerParam(self):
    pass

  def resetEncoders(self):
    self.pitchRelativeEncoder.setPosition(self.pitchAbsoluteEncoder.get())

  def getPitch(self):
    return self.pitchRelativeEncoder.getPosition()
  
  def setGoalPitch(self, pitch: float):
    self.desiredPitch = pitch
    self.pitchClosedLoopController.setReference(pitch, SparkMax.ControlType.kPosition)

  def atGoalPitch(self):
    return math.isclose(self.getPitch(), self.desiredPitch, abs_tol=5)

  def stopPitch(self):
    self.pitchMotor.stopMotor()

  def stopRoller(self):
    pass

  def stop(self):
    self.stopPitch()
    self.stopRoller()

  def periodic(self):
    SmartDashboard.putNumber("Pivot Pitch", self.getPitch())
    SmartDashboard.putNumber("Pivot Pitch Abs", self.pitchAbsoluteEncoder.get())
    if abs(self.getPitch() - self.pitchAbsoluteEncoder.get()) >= 5:
      self.resetEncoders()
      print("hello")
