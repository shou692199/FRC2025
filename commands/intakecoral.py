from commands2 import SequentialCommandGroup, WaitCommand
from subsystems import Shooter

class IntakeCoral(SequentialCommandGroup):
  def __init__(self, shooter: Shooter):
    super().__init__()

    self.shooter = shooter
    self.addCommands(
      shooter.intakeCoralCommand(),
      WaitCommand(5)
    )

    self.addRequirements(shooter)

  def end(self, interrupted):
    self.shooter.stopRoller()
