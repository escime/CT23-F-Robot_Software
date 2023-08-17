import commands2
from rev import CANSparkMax
from constants import IntakeConstants


class IntakeSubsystem(commands2.SubsystemBase):

    m_intake_motor = CANSparkMax(IntakeConstants.motor_id, CANSparkMax.MotorType.kBrushless)
    m_intake_motor.setInverted(False)
    current_state = "inactive"
    go_held = False

    def __init__(self) -> None:
        super().__init__()
        self.current_state = "armed"

    def intake(self, outtake: bool, speed: float) -> None:

        if self.m_intake_motor.getOutputCurrent() < IntakeConstants.current_limit:
            if not outtake:
                self.go_held = False
                self.m_intake_motor.set(speed)
                self.current_state = "intaking"

        if outtake:
            self.m_intake_motor.set(speed * -1)
            self.current_state = "outtaking"
            self.go_held = False

        if self.m_intake_motor.getOutputCurrent() > IntakeConstants.current_limit:
            self.go_held = True
            self.armed()

    def get_go(self) -> bool:
        return self.go_held

    def shoot(self, setpoint: str) -> None:
        if setpoint == "shoot_high_front":
            self.intake(True, IntakeConstants.high_front_power)
        elif setpoint == "shoot_mid_front":
            self.intake(True, IntakeConstants.mid_front_power)
        elif setpoint == "shoot_high_back":
            self.intake(True, IntakeConstants.high_back_power)
        else:
            self.intake(True, IntakeConstants.mid_back_power)

    def armed(self) -> None:
        self.m_intake_motor.set(IntakeConstants.armed_speed)
