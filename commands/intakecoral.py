from commands2 import SequentialCommandGroup, WaitCommand
from subsystems import Shooter, Pivot

class IntakeCoral(SequentialCommandGroup):
  def __init__(self, shooter: Shooter, pivot: Pivot):
    super().__init__()

    self.shooter = shooter
    self.pivot = pivot
    self.addCommands(
      shooter.intakeCoralCommand(),
      pivot.intakeCoralCommand()
    )

    self.addRequirements(shooter, pivot)

  def end(self, interrupted):
    self.shooter.stopRoller()
    self.pivot.stopRoller()
