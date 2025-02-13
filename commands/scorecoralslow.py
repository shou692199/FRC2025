from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from subsystems import Shooter

class ScoreCoralSlow(SequentialCommandGroup):
  def __init__(self, shooter: Shooter):
    super().__init__()

    self.addCommands(
      shooter.outtakeCoralCommand(True),
      WaitCommand(2),
      InstantCommand(shooter.stopRoller)
    )

    self.addRequirements(shooter)