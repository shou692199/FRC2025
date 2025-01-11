import math
from constants import ModuleConstants
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration
from phoenix6.configs.config_groups import SensorDirectionValue
from phoenix6.configs.config_groups import InvertedValue, NeutralModeValue
from phoenix6.controls import DutyCycleOut, NeutralOut, VelocityVoltage
from phoenix6.hardware import CANcoder, TalonFX
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

class SwerveModule:
  def __init__(
    self,
    driveMotorId: int,
    steerMotorId: int,
    shaftEncoderId: int,
    shaftEncoderOffset: float
  ):
    self.driveMotor = TalonFX(driveMotorId, ModuleConstants.kCANbus)
    self.steerMotor = TalonFX(steerMotorId, ModuleConstants.kCANbus)
    self.shaftEncoder = CANcoder(shaftEncoderId, ModuleConstants.kCANbus)
    self.shaftEncoderOffset = shaftEncoderOffset

    self.configureBaseParam()

    self.drivePositionStatus = self.driveMotor.get_position()
    self.driveVelocityStatus = self.driveMotor.get_velocity()
    self.steerPositionStatus = self.steerMotor.get_position()
    self.steerVelocityStatus = self.steerMotor.get_velocity()
    self.shaftPositionStatus = self.shaftEncoder.get_absolute_position()

    self.resetEncoders()
    
    self.driveVelocityVoltage = VelocityVoltage(0).with_slot(0)
    self.steerPIDController = PIDController(ModuleConstants.kPSteerMotor, 0, 0)
    self.steerPIDController.enableContinuousInput(-math.pi, math.pi)
  
  def configureBaseParam(self):
    cfg_drive = TalonFXConfiguration()
    cfg_drive.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    cfg_drive.feedback.sensor_to_mechanism_ratio = 1 / ModuleConstants.kDriveEncoderRot2Meter
    cfg_drive.slot0.k_p = ModuleConstants.kPDriveMotor
    cfg_drive.slot0.k_s = ModuleConstants.kSDriveMotor
    cfg_drive.slot0.k_v = ModuleConstants.kVDriveMotor
    self.driveMotor.configurator.apply(cfg_drive)

    cfg_steer = TalonFXConfiguration()
    cfg_steer.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    cfg_steer.feedback.sensor_to_mechanism_ratio = 1 / ModuleConstants.kSteerEncoderRot2Rad
    cfg_steer.motor_output.neutral_mode = NeutralModeValue.BRAKE
    self.steerMotor.configurator.apply(cfg_steer)

    cfg_shaft = CANcoderConfiguration()
    cfg_shaft.magnet_sensor.absolute_sensor_discontinuity_point = 1
    cfg_shaft.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    cfg_shaft.magnet_sensor.magnet_offset = -self.shaftEncoderOffset
    self.shaftEncoder.configurator.apply(cfg_shaft)
    
  def getDrivePosition(self):
    return self.drivePositionStatus.refresh(False).value_as_double

  def getSteerPosition(self):
    return self.steerPositionStatus.refresh(False).value_as_double
  
  def getDriveVelocity(self):
    return self.driveVelocityStatus.refresh(False).value_as_double

  def getSteerVelocity(self):
    return self.steerVelocityStatus.refresh(False).value_as_double
  
  def getShaftEncoderRad(self):
    return math.tau * self.shaftPositionStatus.refresh(False).value
  
  def resetEncoders(self):
    self.driveMotor.set_position(0)
    self.steerMotor.set_position(self.getShaftEncoderRad())
  
  def getModuleAngle(self):
    return self.getModuleState().angle

  def getModulePosition(self):
    return SwerveModulePosition(self.getDrivePosition(), Rotation2d(self.getSteerPosition()))
  
  def getModuleState(self):
    return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getSteerPosition()))

  def setDesiredState(self, state: SwerveModuleState, identifier: str = ""):
    if state.speed < 0.001:
      self.stop()
      return
    
    state.optimize(self.getModuleAngle())
    self.driveMotor.set_control(self.driveVelocityVoltage.with_velocity(state.speed))
    self.steerMotor.set_control(
      DutyCycleOut(self.steerPIDController.calculate(self.getSteerPosition(), state.angle.radians()))
    )

  def stop(self):
    self.driveMotor.set_control(NeutralOut())
    self.steerMotor.set_control(NeutralOut())