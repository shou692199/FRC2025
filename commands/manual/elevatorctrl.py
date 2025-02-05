import commands2
from subsystems import Elevator
from constants import ElevatorConstants

class ElevatorCtrl(commands2.Command):
  def __init__(
      self,
      elevator: Elevator,
      isUp: bool
  ):
    self.elevator = elevator
    self.isUp = isUp

  def initialize(self):
    self.elevator.setSpeed(
      ElevatorConstants.kTeleMaxDutyCycle * (1 if self.isUp else -1)
    )

  def execute(self):
    if self.isUp:
      print("Elevator Up")
    else:
      print("Elevator Down")

  def end(self, interrupted):
    self.elevator.stop()
    print("Elevator Stopped")

  def isFinished(self):
    return False