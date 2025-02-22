import math
import commands2
from wpilib import SmartDashboard
from rev import SparkMax, SparkMaxConfig, ClosedLoopConfig
from constants import ClimberConstants

class Climber(commands2.Subsystem):
  def __init__(self):
    self.motor = SparkMax(
      ClimberConstants.kMotorId, SparkMax.MotorType.kBrushless
    )
    self.motorSub = SparkMax(
      ClimberConstants.kMotorSubId, SparkMax.MotorType.kBrushless
    )

    self.encoder = self.motor.getAbsoluteEncoder()
    self.closedLoopController = self.motor.getClosedLoopController()

    cfg = SparkMaxConfig()
    cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    cfg.smartCurrentLimit(ClimberConstants.kSmartCurrentLimit)

    cfg.absoluteEncoder.zeroCentered(True)
    cfg.absoluteEncoder.zeroOffset(ClimberConstants.kAbsoluteEncoderOffset)
    cfg.absoluteEncoder.positionConversionFactor(360)

    cfg.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    cfg.closedLoop.P(0.1)
    cfg.closedLoop.outputRange(-0.2, 0.4)

    self.motor.configure(
      cfg,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )

    self.motorSub.configure(
      SparkMaxConfig().apply(cfg).follow(self.motor, True),
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kPersistParameters
    )
  
  def getAngle(self):
    return self.encoder.getPosition()
  
  def setGoalAngle(self, angle: float):
    self.desiredAngle = angle
    self.closedLoopController.setReference(angle, SparkMax.ControlType.kPosition)

  def atGoalAngle(self):
    return math.isclose(self.getAngle(), self.desiredAngle, abs_tol=5)

  def setSpeed(self, speed: float):
    if speed < 0.001:
      #self.setGoalAngle(self.getAngle())
      return
    self.motor.set(speed)

  def stop(self):
    self.motor.stopMotor()

  def periodic(self):
    SmartDashboard.putNumber("Climber Angle", self.getAngle())