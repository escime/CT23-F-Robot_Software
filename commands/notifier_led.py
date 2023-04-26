import commands2
from subsystems.leds import LEDs


class NotifierLEDs(commands2.CommandBase):
    def __init__(self, leds: LEDs, color: str, current: str):
        super().__init__()
        self.leds = leds
        self.addRequirements(leds)
        self.color = color
        self.current = current
        self.addRequirements(leds)

    def initialize(self) -> None:
        self.leds.set_notifier(self.color)
        if self.current == "purple_chaser":
            self.leds.purple_chaser()
        elif self.current == "rainbow_shift":
            self.leds.rainbow_shift()
        else:
            self.leds.fire([170, 0, 255], False)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
