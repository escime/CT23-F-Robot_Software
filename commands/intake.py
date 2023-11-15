import commands2
from subsystems.intakesubsystem import IntakeSubsystem


class Intake(commands2.Command):
    def __init__(self, intake: IntakeSubsystem, outtake: bool, speed: float):
        super().__init__()
        self.intake = intake
        self.outtake = outtake
        self.speed = speed
        self.addRequirements(intake)

    def execute(self) -> None:
        self.intake.intake(self.outtake, self.speed)

    def isFinished(self) -> bool:
        return True
