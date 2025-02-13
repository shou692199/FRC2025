from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from subsystems import Shooter

class IntakeAlgae(SequentialCommandGroup):
  def __init__(self, shooter: Shooter):
    super().__init__()

    self.addCommands(
      shooter.intakeAlgaeCommand(),
      WaitCommand(5),
      InstantCommand(shooter.holdRoller)
    )

    self.addRequirements(shooter)