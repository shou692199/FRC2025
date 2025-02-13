from commands2 import Command
from subsystems import Climber

class ReleaseClimber(Command):
  def __init__(self, climber: Climber):
    self.climber = climber

    self.addRequirements(self.climber)
    
  def initialize(self):
    self.climber.setGoalAngle(-7)

  def isFinished(self):
    return self.climber.atGoalAngle()
