from commands2 import Command
from subsystems import Pivot

class SetPivotPitch(Command):
  def __init__(self, pivot: Pivot, pitch: float):
    self.pivot = pivot
    self.pitch = pitch

    self.addRequirements(self.pivot)
    
  def initialize(self):
    self.pivot.setGoalPitch(self.pitch)

  def isFinished(self):
    return self.pivot.atGoalPitch()

  