import math
import typing
from wpilib.simulation import DCMotorSim, SimDeviceSim, ElevatorSim
from wpilib import RobotController, DriverStation, SmartDashboard, Mechanism2d
from wpimath.units import radiansToRotations, metersToFeet
from wpimath.system.plant import DCMotor, LinearSystemId
from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged
from rev import SparkMaxSim
from swervemodule import SwerveModule
from subsystems.swerve import Swerve
from subsystems.elevator import Elevator
from constants import DriveConstants, ElevatorConstants

if typing.TYPE_CHECKING:
  from robot import MyRobot

class SwerveModulePhysics:
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

class SwervePhysics:
  kWheelBaseFeet = metersToFeet(DriveConstants.kWheelBaseMeters)
  kPhysicalMaxSpeedFeetPerSecond = metersToFeet(DriveConstants.kPhysicalMaxSpeedMetersPerSecond)

  def __init__(self, physicsController: PhysicsInterface, swerve: Swerve):
    self.physicsController = physicsController
    self.swerve = swerve
    self.modulesSim = [SwerveModulePhysics(m) for m in swerve.modules]
    self.gyroAngleSim = SimDeviceSim("navX-Sensor[4]").getDouble("Yaw")
  
  def drive(self, tm_diff: float):
    for m in self.modulesSim:
      m.update(tm_diff)

    chassisSpeeds = self.swerve.getChassisSpeeds()
    pose = self.physicsController.drive(chassisSpeeds, tm_diff)
    if abs(chassisSpeeds.omega) > 0.001:
      self.gyroAngleSim.set(-pose.rotation().degrees())

class ElevatorPhysics:
  def __init__(self, physicsController: PhysicsInterface, elevator: Elevator):
    self.physicsController = physicsController
    self.elevator = elevator

    gearbox = DCMotor.NEO(2)
    self.elevatorSim = ElevatorSim(
      gearbox,
      1 / ElevatorConstants.kLiftMotorGearRatio,
      5,
      ElevatorConstants.kChainPitchMeters * ElevatorConstants.kChainWheelTeeth / math.pi,
      ElevatorConstants.kReverseLimitMeters,
      ElevatorConstants.kForwardLimitMeters,
      False,
      0,
      [0.01, 0.0]
    )
    self.liftMotorSim = SparkMaxSim(elevator.liftMotor, gearbox)

    self.mech2d = Mechanism2d(20, 50)
    self.elevatorRoot = self.mech2d.getRoot("Elevator Root", 10, 0)
    self.elevatorMech2d = self.elevatorRoot.appendLigament(
      "Elevator", self.elevatorSim.getPositionInches(), 90
    )
    SmartDashboard.putData("Elevator Sim", self.mech2d)

  def update(self, tm_diff: float):
    batteryVoltage = RobotController.getBatteryVoltage()
    self.elevatorSim.setInput(
      0, self.liftMotorSim.getAppliedOutput() * batteryVoltage
    )
    self.elevatorSim.update(tm_diff)

    self.liftMotorSim.iterate(self.elevatorSim.getVelocity(), batteryVoltage, tm_diff)
    self.elevatorMech2d.setLength(self.elevatorSim.getPositionInches())

class PhysicsEngine:
  def __init__(
    self, physics_controller: PhysicsInterface, robot: "MyRobot"
  ):
    self.swerveSim = SwervePhysics(physics_controller, robot.swerve)
    self.elevatorSim = ElevatorPhysics(physics_controller, robot.elevator)
  
  def update_sim(self, now: float, tm_diff: float):
    if DriverStation.isEnabled():
      unmanaged.feed_enable(100)

    self.swerveSim.drive(tm_diff)
    self.elevatorSim.update(tm_diff)
