import commands2
from subsystems.visionsubsystem import VisionSubsystem


class ToggleLLLEDs(commands2.CommandBase):
    def __init__(self, ll: VisionSubsystem):
        super().__init__()
        self.ll = ll
        self.addRequirements(ll)

    def execute(self) -> None:
        self.ll_toggle_leds(True)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
