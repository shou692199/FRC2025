import math
import typing
from wpilib.simulation import DCMotorSim, SimDeviceSim
from wpilib import RobotController, DriverStation
from wpimath.units import radiansToRotations, metersToInches
from wpimath.system.plant import DCMotor, LinearSystemId
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
from phoenix6 import unmanaged
from phoenix6.sim import ChassisReference
from constants import ModuleConstants, DriveConstants

if typing.TYPE_CHECKING:
  from robot import MyRobot

kWheelBase = metersToInches(DriveConstants.kWheelBase)

class PhysicsEngine:
  def __init__(
    self, physics_controller: PhysicsInterface, robot: "MyRobot"
  ):
    self.physics_controller = physics_controller
    self.swerve = robot.swerve

    self.fld_talon_sim = self.swerve.frontLeft.driveMotor.sim_state
    self.fls_talon_sim = self.swerve.frontLeft.steerMotor.sim_state
    self.frd_talon_sim = self.swerve.frontRight.driveMotor.sim_state
    self.frs_talon_sim = self.swerve.frontRight.steerMotor.sim_state
    self.bld_talon_sim = self.swerve.backLeft.driveMotor.sim_state
    self.bls_talon_sim = self.swerve.backLeft.steerMotor.sim_state
    self.brd_talon_sim = self.swerve.backRight.driveMotor.sim_state
    self.brs_talon_sim = self.swerve.backRight.steerMotor.sim_state

    gearbox = DCMotor.falcon500(1)
    self.fld_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)
    self.fls_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)
    self.frd_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)
    self.frs_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)
    self.bld_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)
    self.bls_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)
    self.brd_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)
    self.brs_motor_sim = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.001, 1), gearbox)

    #self.fld_talon_sim.orientation = ChassisReference.Clockwise_Positive
    #self.frd_talon_sim.orientation = ChassisReference.Clockwise_Positive
    #self.bld_talon_sim.orientation = ChassisReference.Clockwise_Positive
    #self.brd_talon_sim.orientation = ChassisReference.Clockwise_Positive

    self.fl_shaft_sim = self.swerve.frontLeft.shaftEncoder.sim_state
    self.fr_shaft_sim = self.swerve.frontRight.shaftEncoder.sim_state
    self.bl_shaft_sim = self.swerve.backLeft.shaftEncoder.sim_state
    self.br_shaft_sim = self.swerve.backRight.shaftEncoder.sim_state

    self.fl_shaft_init = self.swerve.frontLeft.getShaftEncoderRad()
    self.fr_shaft_init = self.swerve.frontRight.getShaftEncoderRad()
    self.bl_shaft_init = self.swerve.backLeft.getShaftEncoderRad()
    self.br_shaft_init = self.swerve.backRight.getShaftEncoderRad()

    self.fls_offset = self.fl_shaft_init / ModuleConstants.kSteerEncoderRot2Rad
    self.frs_offset = self.fr_shaft_init / ModuleConstants.kSteerEncoderRot2Rad
    self.bls_offset = self.bl_shaft_init / ModuleConstants.kSteerEncoderRot2Rad
    self.brs_offset = self.br_shaft_init / ModuleConstants.kSteerEncoderRot2Rad

    self.fls_position = self.swerve.frontLeft.getSteerPosition
    self.frs_position = self.swerve.frontRight.getSteerPosition
    self.bls_position = self.swerve.backLeft.getSteerPosition
    self.brs_position = self.swerve.backRight.getSteerPosition

    self.fls_state = self.swerve.frontLeft.getModuleState
    self.frs_state = self.swerve.frontRight.getModuleState
    self.bls_state = self.swerve.backLeft.getModuleState
    self.brs_state = self.swerve.backRight.getModuleState

    self.fld_duty_cycle = self.swerve.frontLeft.driveMotor.get_duty_cycle()
    self.fls_duty_cycle = self.swerve.frontLeft.steerMotor.get_duty_cycle()
    self.frd_duty_cycle = self.swerve.frontRight.driveMotor.get_duty_cycle()
    self.frs_duty_cycle = self.swerve.frontRight.steerMotor.get_duty_cycle()
    self.bld_duty_cycle = self.swerve.backLeft.driveMotor.get_duty_cycle()
    self.bls_duty_cycle = self.swerve.backLeft.steerMotor.get_duty_cycle()
    self.brd_duty_cycle = self.swerve.backRight.driveMotor.get_duty_cycle()
    self.brs_duty_cycle = self.swerve.backRight.steerMotor.get_duty_cycle()
    
    self.gyroAngle = SimDeviceSim("navX-Sensor[4]").getDouble("Yaw")
  
  def update_sim(self, now: float, tm_diff: float):
    if DriverStation.isEnabled():
      unmanaged.feed_enable(100)

    battery_voltage = RobotController.getBatteryVoltage()
    self.fld_talon_sim.set_supply_voltage(battery_voltage)
    self.fls_talon_sim.set_supply_voltage(battery_voltage)
    self.frd_talon_sim.set_supply_voltage(battery_voltage)
    self.frs_talon_sim.set_supply_voltage(battery_voltage)
    self.bld_talon_sim.set_supply_voltage(battery_voltage)
    self.bls_talon_sim.set_supply_voltage(battery_voltage)
    self.brd_talon_sim.set_supply_voltage(battery_voltage)
    self.brs_talon_sim.set_supply_voltage(battery_voltage)
    
    self.fld_motor_sim.setInputVoltage(self.fld_talon_sim.motor_voltage)
    self.fls_motor_sim.setInputVoltage(self.fls_talon_sim.motor_voltage)
    self.frd_motor_sim.setInputVoltage(self.frd_talon_sim.motor_voltage)
    self.frs_motor_sim.setInputVoltage(self.frs_talon_sim.motor_voltage)
    self.bld_motor_sim.setInputVoltage(self.bld_talon_sim.motor_voltage)
    self.bls_motor_sim.setInputVoltage(self.bls_talon_sim.motor_voltage)
    self.brd_motor_sim.setInputVoltage(self.brd_talon_sim.motor_voltage)
    self.brs_motor_sim.setInputVoltage(self.brs_talon_sim.motor_voltage)

    self.fld_motor_sim.update(tm_diff)
    self.fls_motor_sim.update(tm_diff)
    self.frd_motor_sim.update(tm_diff)
    self.frs_motor_sim.update(tm_diff)
    self.bld_motor_sim.update(tm_diff)
    self.bls_motor_sim.update(tm_diff)
    self.brd_motor_sim.update(tm_diff)
    self.brs_motor_sim.update(tm_diff)

    self.fld_talon_sim.set_raw_rotor_position(radiansToRotations(self.fld_motor_sim.getAngularPosition()))
    self.fls_talon_sim.set_raw_rotor_position(radiansToRotations(self.fls_motor_sim.getAngularPosition()))# - self.fls_offset)
    self.frd_talon_sim.set_raw_rotor_position(radiansToRotations(self.frd_motor_sim.getAngularPosition()))
    self.frs_talon_sim.set_raw_rotor_position(radiansToRotations(self.frs_motor_sim.getAngularPosition()))# - self.frs_offset)
    self.bld_talon_sim.set_raw_rotor_position(radiansToRotations(self.bld_motor_sim.getAngularPosition()))
    self.bls_talon_sim.set_raw_rotor_position(radiansToRotations(self.bls_motor_sim.getAngularPosition()))# - self.bls_offset)
    self.brd_talon_sim.set_raw_rotor_position(radiansToRotations(self.brd_motor_sim.getAngularPosition()))
    self.brs_talon_sim.set_raw_rotor_position(radiansToRotations(self.brs_motor_sim.getAngularPosition()))# - self.brs_offset)

    self.fl_shaft_sim.set_raw_position((self.fls_position() - self.fl_shaft_init) / math.tau)
    self.fr_shaft_sim.set_raw_position((self.frs_position() - self.fr_shaft_init) / math.tau)
    self.bl_shaft_sim.set_raw_position((self.bls_position() - self.bl_shaft_init) / math.tau)
    self.br_shaft_sim.set_raw_position((self.brs_position() - self.br_shaft_init) / math.tau)

    self.fld_talon_sim.set_rotor_velocity(radiansToRotations(self.fld_motor_sim.getAngularVelocity()))
    self.fls_talon_sim.set_rotor_velocity(radiansToRotations(self.fls_motor_sim.getAngularVelocity()))
    self.frd_talon_sim.set_rotor_velocity(radiansToRotations(self.frd_motor_sim.getAngularVelocity()))
    self.frs_talon_sim.set_rotor_velocity(radiansToRotations(self.frs_motor_sim.getAngularVelocity()))
    self.bld_talon_sim.set_rotor_velocity(radiansToRotations(self.bld_motor_sim.getAngularVelocity()))
    self.bls_talon_sim.set_rotor_velocity(radiansToRotations(self.bls_motor_sim.getAngularVelocity()))
    self.brd_talon_sim.set_rotor_velocity(radiansToRotations(self.brd_motor_sim.getAngularVelocity()))
    self.brs_talon_sim.set_rotor_velocity(radiansToRotations(self.brs_motor_sim.getAngularVelocity()))

    speeds = drivetrains.four_motor_swerve_drivetrain(
      self.bld_duty_cycle.refresh(False).value_as_double,
      self.brd_duty_cycle.refresh(False).value_as_double,
      self.fld_duty_cycle.refresh(False).value_as_double,
      self.frd_duty_cycle.refresh(False).value_as_double,
      (-self.swerve.backLeft.getModuleAngle().degrees() + 360) % 360,
      (-self.swerve.backRight.getModuleAngle().degrees() + 360) % 360,
      (-self.swerve.frontLeft.getModuleAngle().degrees() + 360) % 360,
      (-self.swerve.frontRight.getModuleAngle().degrees() + 360) % 360,
      kWheelBase,
      kWheelBase
    )
    pose = self.physics_controller.drive(speeds, tm_diff)

    self.gyroAngle.set(-pose.rotation().degrees())