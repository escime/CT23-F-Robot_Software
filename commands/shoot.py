import commands2
from subsystems.intakesubsystem import IntakeSubsystem


class Shoot(commands2.Command):
    def __init__(self, intake: IntakeSubsystem, setpoint: str):
        super().__init__()
        self.intake = intake
        self.setpoint = setpoint
        self.addRequirements(intake)

    def execute(self) -> None:
        self.intake.shoot(self.setpoint)

    def isFinished(self) -> bool:
        return True
