import commands2
from subsystems.drivesubsystem import DriveSubsystem
from constants import DriveConstants


class DriveToCS(commands2.Command):
    def __init__(self, robot_drive: DriveSubsystem, direction: int):
        super().__init__()
        self.robot_drive = robot_drive
        self.direction = direction
        self.addRequirements(robot_drive)

    def execute(self) -> None:
        if self.direction == 0:
            self.robot_drive.snap_drive(0.5 * DriveConstants.kMaxSpeed, 0, 0)
        else:
            self.robot_drive.snap_drive(-0.5, 0, 180)

    def isFinished(self) -> bool:
        if not self.robot_drive.balanced:
            return True
        else:
            return False
