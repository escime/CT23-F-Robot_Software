from commands2 import Command, CommandScheduler, TimedCommandRobot
from robotcontainer import RobotContainer
from wpilib import run


class Robot(TimedCommandRobot):
    m_autonomous_command: Command
    m_robotcontainer: RobotContainer

    def robotInit(self) -> None:
        self.m_robotcontainer = RobotContainer()
        self.m_autonomous_command = None

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """Oops?"""

    def disabledPeriodic(self) -> None:
        """Doing my best"""

    def autonomousInit(self) -> None:
        self.m_autonomous_command = self.m_robotcontainer.getAutonomousCommand()

        if self.m_autonomous_command is not None:
            self.m_autonomous_command.schedule()

    def teleopInit(self) -> None:
        if self.m_autonomous_command:
            self.m_autonomous_command.cancel()

    def teleopPeriodic(self) -> None:
        """AGH"""

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    run(Robot)
