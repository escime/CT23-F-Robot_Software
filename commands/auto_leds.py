import commands2
from subsystems.leds import LEDs
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem


class AutoLEDs(commands2.Command):
    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, leds: LEDs):
        super().__init__()
        self.leds = leds
        self.drive = drive
        self.arm = arm
        self.addRequirements(leds, arm, drive)

    def initialize(self) -> None:
        self.leds.auto_set_check(self.drive, self.arm)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
