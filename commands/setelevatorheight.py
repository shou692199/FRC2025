from commands2 import Command
from subsystems import Elevator
from constants import MotionPresets

class SetElevatorHeight(Command):
  def __init__(self, elevator: Elevator, preset: MotionPresets):
    self.elevator = elevator
    self.preset = preset

    self.addRequirements(self.elevator)
    
  def initialize(self):
    self.elevator.setGoalHeight(self.preset.value[0])

  def isFinished(self):
    return self.elevator.atGoalHeight()

  