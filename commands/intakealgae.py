from commands2 import SequentialCommandGroup, WaitCommand
from subsystems import Shooter, Pivot

class IntakeAlgae(SequentialCommandGroup):
  def __init__(self, shooter: Shooter, pivot: Pivot, shortPeriod = False):
    super().__init__()

    self.addCommands(
      shooter.intakeAlgaeCommand(),
      pivot.intakeAlgaeCommand(),
      WaitCommand(3 if shortPeriod else 6),
      shooter.holdAlgaeCommand()
    )

    self.addRequirements(shooter, pivot)