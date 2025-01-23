import math
import typing
from wpilib.simulation import DCMotorSim, SimDeviceSim
from wpilib import RobotController, DriverStation
from wpimath.units import radiansToRotations, metersToFeet
from wpimath.system.plant import DCMotor, LinearSystemId
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
from phoenix6 import unmanaged
from swervemodule import SwerveModule
from subsystems.swerve import Swerve
from constants import DriveConstants

if typing.TYPE_CHECKING:
  from robot import MyRobot

class SwerveModuleSim:
  def __init__(self, module: SwerveModule):
    self.module = module
    self.driveMotorSim = module.driveMotor.sim_state
    self.steerMotorSim = module.steerMotor.sim_state
    self.shaftEncoderSim = module.shaftEncoder.sim_state
    self.shaftInitialPos = module.getShaftEncoderRad()

    gearbox = DCMotor.falcon500(1)
    self.driveMotorSysId = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.0001, 1), gearbox)
    self.steerMotorSysId = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.0001, 1), gearbox)

  def update(self, tm_diff: float):
    batteryVoltage = RobotController.getBatteryVoltage()
    self.driveMotorSim.set_supply_voltage(batteryVoltage)
    self.steerMotorSim.set_supply_voltage(batteryVoltage)
    self.driveMotorSysId.setInputVoltage(self.driveMotorSim.motor_voltage)
    self.steerMotorSysId.setInputVoltage(self.steerMotorSim.motor_voltage)

    self.driveMotorSysId.update(tm_diff)
    self.steerMotorSysId.update(tm_diff)
    self.updateKinematic()

  def updateKinematic(self):
    self.driveMotorSim.set_raw_rotor_position(radiansToRotations(self.driveMotorSysId.getAngularPosition()))
    self.steerMotorSim.set_raw_rotor_position(radiansToRotations(self.steerMotorSysId.getAngularPosition()))
    self.shaftEncoderSim.set_raw_position((self.module.getSteerPosition() - self.shaftInitialPos) / math.tau)

    self.driveMotorSim.set_rotor_velocity(radiansToRotations(self.driveMotorSysId.getAngularVelocity()))
    self.steerMotorSim.set_rotor_velocity(radiansToRotations(self.steerMotorSysId.getAngularVelocity()))

  def getDriveDutyCycle(self):
    return self.module.driveMotor.get_duty_cycle().value_as_double

class SwerveSim:
  kWheelBaseFeet = metersToFeet(DriveConstants.kWheelBaseMeters)
  kPhysicalMaxSpeedFeetPerSecond = metersToFeet(DriveConstants.kPhysicalMaxSpeedMetersPerSecond)

  def __init__(self, physicsController: PhysicsInterface, swerve: Swerve):
    self.physicsController = physicsController
    self.modules = [swerve.backLeft, swerve.backRight, swerve.frontLeft, swerve.frontRight]
    self.modulesSim = [SwerveModuleSim(m) for m in self.modules]
    self.gyroAngleSim = SimDeviceSim("navX-Sensor[4]").getDouble("Yaw")
  
  def drive(self, tm_diff: float):
    for m in self.modulesSim:
      m.update(tm_diff)

    chassisSpeeds = drivetrains.four_motor_swerve_drivetrain(
      self.modulesSim[0].getDriveDutyCycle(),
      self.modulesSim[1].getDriveDutyCycle(),
      self.modulesSim[2].getDriveDutyCycle(),
      self.modulesSim[3].getDriveDutyCycle(),
      (-self.modules[0].getModuleAngle().degrees() + 360) % 360,
      (-self.modules[1].getModuleAngle().degrees() + 360) % 360,
      (-self.modules[2].getModuleAngle().degrees() + 360) % 360,
      (-self.modules[3].getModuleAngle().degrees() + 360) % 360,
      SwerveSim.kWheelBaseFeet,
      SwerveSim.kWheelBaseFeet,
      SwerveSim.kPhysicalMaxSpeedFeetPerSecond
    )

    pose = self.physicsController.drive(chassisSpeeds, tm_diff)
    self.gyroAngleSim.set(-pose.rotation().degrees())

class PhysicsEngine:
  def __init__(
    self, physics_controller: PhysicsInterface, robot: "MyRobot"
  ):
    self.swerveSim = SwerveSim(physics_controller, robot.swerve)
  
  def update_sim(self, now: float, tm_diff: float):
    if DriverStation.isEnabled():
      unmanaged.feed_enable(100)

    self.swerveSim.drive(tm_diff)
