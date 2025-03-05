from typing import Callable
from wpimath import applyDeadband
from commands2 import Command
from subsystems import Climber

class ClimbJoystick(Command):
  def __init__(self, climber: Climber, speed: Callable[[], float]):
    self.climber = climber
    self.speed = speed

    self.addRequirements(self.climber)
    
  def initialize(self):
    self.climber.setSpeed(0.2)

  def execute(self):
    self.climber.setSpeed(applyDeadband(self.speed(), 0.05))

  def end(self, interrupted):
    self.climber.stop()

  def isFinished(self):
    return False
