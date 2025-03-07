from commands2 import SequentialCommandGroup, WaitCommand
from subsystems import Shooter, Pivot

class ScoreCoralSlow(SequentialCommandGroup):
  def __init__(self, shooter: Shooter, pivot: Pivot):
    super().__init__()

    self.shooter = shooter
    self.pivot = pivot
    self.addCommands(
      shooter.outtakeCoralCommand(True),
      pivot.outtakeCoralCommand(),
      WaitCommand(2)
    )

    self.addRequirements(shooter, pivot)

  def end(self, interrupted):
    self.shooter.stopRoller()
    self.pivot.stopRoller()