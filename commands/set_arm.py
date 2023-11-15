import commands2
from subsystems.armsubsystem import ArmSubsystem


class SetArm(commands2.Command):
    def __init__(self, arm: ArmSubsystem, setpoint: str):
        super().__init__()
        self.arm = arm
        self.setpoint = setpoint
        self.addRequirements(arm)

    def execute(self) -> None:
        self.arm.set_setpoint(self.setpoint)

    def isFinished(self) -> bool:
        return self.arm.check_completion()
