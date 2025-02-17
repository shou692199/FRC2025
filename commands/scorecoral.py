from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from subsystems import Shooter

class ScoreCoral(SequentialCommandGroup):
  def __init__(self, shooter: Shooter):
    super().__init__()

    self.shooter = shooter
    self.addCommands(
      shooter.outtakeCoralCommand(),
      WaitCommand(1.5)
    )

    self.addRequirements(shooter)

  def end(self, interrupted):
    self.shooter.stopRoller()