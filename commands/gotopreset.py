from commands2 import SequentialCommandGroup
from subsystems import Elevator, Shooter, Pivot
from commands.setelevatorheight import SetElevatorHeight
from commands.setpivotpitch import SetPivotPitch
from commands.setshooterpitch import SetShooterPitch
from constants import MotionPresets

class GotoPreset(SequentialCommandGroup):
  def __init__(self, elevator: Elevator, shooter: Shooter, pivot: Pivot, preset: MotionPresets):
    super().__init__()

    self.addCommands(
      SetPivotPitch(pivot, 165).withTimeout(2),
      SetShooterPitch(shooter, 0).withTimeout(2),
      SetElevatorHeight(elevator, preset.value[0]).withTimeout(3),
      SetPivotPitch(pivot, preset.value[1]).withTimeout(2),
      SetShooterPitch(shooter, preset.value[2]).withTimeout(1)
    )
