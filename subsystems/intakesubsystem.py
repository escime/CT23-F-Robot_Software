import commands2
from rev import CANSparkMax
from constants import IntakeConstants
from subsystems.armsubsystem import ArmSubsystem


class IntakeSubsystem(commands2.SubsystemBase):

    m_intake_motor = CANSparkMax(IntakeConstants.motor_id, CANSparkMax.MotorType.kBrushless)
    m_intake_motor.setInverted(False)
    current_state = "inactive"
    go_held = False

    def __init__(self) -> None:
        super().__init__()
        self.current_state = "armed"

    def intake(self, outtake: bool, speed: float) -> None:
        """Handle intake I/O with over-current protection."""

        if self.m_intake_motor.getOutputCurrent() < IntakeConstants.current_limit and self.current_state is not "armed":
            if not outtake:
                self.go_held = False
                self.m_intake_motor.set(speed)
                self.current_state = "intaking"

        if outtake:
            self.m_intake_motor.set(speed * -1)
            self.current_state = "outtaking"
            self.go_held = False

        if self.m_intake_motor.getOutputCurrent() > IntakeConstants.current_limit or self.current_state is "armed":
            self.go_held = True
            self.current_state = "armed"
            self.arm()

    def get_go(self) -> bool:
        """Get whether the robot thinks it has a game object."""
        return self.go_held

    def shoot(self, setpoint: str) -> None:
        """Auto-setter for shooter tunings."""
        if setpoint == "shoot_high_front":
            self.intake(True, IntakeConstants.high_front_power)
        elif setpoint == "shoot_mid_front":
            self.intake(True, IntakeConstants.mid_front_power)
        elif setpoint == "shoot_high_back":
            self.intake(True, IntakeConstants.high_back_power)
        elif setpoint == "shoot_mid_back":
            self.intake(True, IntakeConstants.mid_back_power)

    def arm(self) -> None:
        """Set the intake into the "armed" state where it attempts to keep the game object in grip."""
        self.m_intake_motor.set(IntakeConstants.armed_speed)

    def bound_shoot(self, arm: ArmSubsystem):
        """Shoot based on arm setpoint."""
        self.shoot(arm.get_setpoint())
