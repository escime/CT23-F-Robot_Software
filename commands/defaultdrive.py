import typing
import commands2
from subsystems.drivesubsystem import DriveSubsystem


class DefaultDrive(commands2.Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        x_speed: typing.Callable[[], float],
        y_speed: typing.Callable[[], float],
        rot: typing.Callable[[], float],
        field_relative: bool,
        teleop: bool
    ):
        super().__init__()

        self.drive = drive
        self.x_speed = x_speed
        self.y_speed = y_speed
        self.rot = rot
        self.field_relative = field_relative
        self.teleop = teleop

        self.addRequirements(self.drive)

    def initialize(self) -> None:
        self.drive.drive(self.x_speed, self.y_speed, self.rot, self.field_relative, self.teleop)

    def isFinished(self) -> bool:
        return True
