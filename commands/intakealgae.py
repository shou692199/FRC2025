from commands2 import SequentialCommandGroup, WaitCommand
from subsystems import Shooter

class IntakeAlgae(SequentialCommandGroup):
  def __init__(self, shooter: Shooter, shortPeriod = False):
    super().__init__()

    self.addCommands(
      shooter.intakeAlgaeCommand(),
      WaitCommand(3 if shortPeriod else 6),
      shooter.holdAlgaeCommand()
    )

    self.addRequirements(shooter)