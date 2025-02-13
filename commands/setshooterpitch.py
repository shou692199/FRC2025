from commands2 import Command
from subsystems import Shooter
from constants import MotionPresets

class SetShooterPitch(Command):
  def __init__(self, shooter: Shooter, preset: MotionPresets):
    self.shooter = shooter
    self.preset = preset

    self.addRequirements(self.shooter)
    
  def initialize(self):
    self.shooter.setGoalPitch(self.preset.value[1])

  def isFinished(self):
    return self.shooter.atGoalPitch()

  