from commands2 import Command
from subsystems import Climber

class ReleaseClimber(Command):
  def __init__(self, climber: Climber):
    self.climber = climber

    self.addRequirements(self.climber)
    
  def initialize(self):
    self.climber.setSpeed(-0.2)

  def end(self, interrupted):
    self.climber.stop()

  def isFinished(self):
    return False#self.climber.atGoalAngle()
