from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from subsystems import Shooter

class ShootAlgae(SequentialCommandGroup):
  def __init__(self, shooter: Shooter):
    super().__init__()

    self.addCommands(
      shooter.outtakeAlgaeCommand(),
      WaitCommand(3),
      InstantCommand(shooter.stopRoller)
    )

    self.addRequirements(shooter)