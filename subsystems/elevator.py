import commands2
from wpilib import SmartDashboard
from rev import SparkMax, SparkMaxConfig, LimitSwitchConfig, ClosedLoopConfig, MAXMotionConfig
from constants import ElevatorConstants, ReefLayers

class Elevator(commands2.Subsystem):
  def __init__(self):
    self.liftMotor = SparkMax(
      ElevatorConstants.kLiftMotorId, SparkMax.MotorType.kBrushless
    )
    self.liftMotorSub = SparkMax(
      ElevatorConstants.kLiftMotorSubId, SparkMax.MotorType.kBrushless
    )

    self.liftEncoder = self.liftMotor.getEncoder()
    self.liftClosedLoopController = self.liftMotor.getClosedLoopController()

    cfg_lift = SparkMaxConfig()
    cfg_lift.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    cfg_lift.smartCurrentLimit(ElevatorConstants.kSmartCurrentLimit)

    cfg_lift.encoder.positionConversionFactor(ElevatorConstants.kLiftEncoderRot2Meter)
    cfg_lift.encoder.velocityConversionFactor(ElevatorConstants.kLiftEncoderRot2Meter / 60)

    cfg_lift.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
    cfg_lift.limitSwitch.forwardLimitSwitchEnabled(True)
    cfg_lift.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
    cfg_lift.limitSwitch.reverseLimitSwitchEnabled(True)
    cfg_lift.softLimit.forwardSoftLimit(ElevatorConstants.kForwardLimitMeters)
    cfg_lift.softLimit.forwardSoftLimitEnabled(True)
    cfg_lift.softLimit.reverseSoftLimit(ElevatorConstants.kReverseLimitMeters)
    cfg_lift.softLimit.reverseSoftLimitEnabled(True)

    cfg_lift.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    cfg_lift.closedLoop.pid(ElevatorConstants.kPLiftMotor, 0, 0)
    cfg_lift.closedLoop.outputRange(
      -ElevatorConstants.kAutoMaxDutyCycle, ElevatorConstants.kAutoMaxDutyCycle
    )

    self.liftMotor.configure(
      cfg_lift,
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kNoPersistParameters
    )

    self.liftMotorSub.configure(
      SparkMaxConfig().apply(cfg_lift).follow(self.liftMotor, True),
      SparkMax.ResetMode.kResetSafeParameters,
      SparkMax.PersistMode.kNoPersistParameters
    )

    self.resetEncoders()

  def resetEncoders(self):
    self.liftEncoder.setPosition(0)

  def getHeight(self):
    return self.liftEncoder.getPosition()

  def setHeight(self, height: float | ReefLayers):
    if type(height) is ReefLayers:
      height = height.value
    self.liftClosedLoopController.setReference(
      height, SparkMax.ControlType.kPosition
    )

  def setSpeed(self, speed: float):
    self.liftMotor.set(speed)

  def stop(self):
    self.liftMotor.set(0)

  def periodic(self):
    SmartDashboard.putNumber("Elevator Height", self.getHeight())