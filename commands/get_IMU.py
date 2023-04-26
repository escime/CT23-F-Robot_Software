import commands2
from subsystems.drivesubsystem import DriveSubsystem
from wpilib import SmartDashboard


class GetIMU(commands2.CommandBase):
    def __init__(self, robot_drive: DriveSubsystem):
        super().__init__()
        self.robot_drive = robot_drive
        self.addRequirements(robot_drive)

    def initialize(self) -> None:
        SmartDashboard.putData("Heading", self.robot_drive.get_heading())
        print("Hello?")

    def execute(self) -> None:
        SmartDashboard.putData("Heading", self.robot_drive.get_heading())
        print("Hello2?")

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
