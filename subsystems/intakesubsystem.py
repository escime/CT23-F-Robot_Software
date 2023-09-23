import commands2
from rev import CANSparkMax
from constants import IntakeConstants
from subsystems.armsubsystem import ArmSubsystem
from wpilib import DigitalInput
from wpilib import SmartDashboard


class IntakeSubsystem(commands2.SubsystemBase):

    m_intake_motor = CANSparkMax(IntakeConstants.motor_id, CANSparkMax.MotorType.kBrushless)
    m_intake_motor.setInverted(False)
    current_state = "stow"
    go_held = False

    def __init__(self) -> None:
        super().__init__()
        self.current_state = "stow"
        self.sensor = DigitalInput(0)

    def intake(self, outtake: bool, speed: float) -> None:
        """Handle intake I/O with over-current protection."""

        # if self.m_intake_motor.getOutputCurrent() < IntakeConstants.current_limit and self.current_state != "armed":
        # if not outtake and self.sensor.get():
        if not outtake:
            self.m_intake_motor.set(speed)
            self.current_state = "intaking"

        if not outtake and not self.sensor.get():
            self.m_intake_motor.set(speed * 0.05)
            self.current_state = "holding"

        if outtake:
            self.m_intake_motor.set(speed * -1)
            self.current_state = "outtaking"

        # if self.m_intake_motor.getOutputCurrent() > IntakeConstants.current_limit or self.current_state == "armed":
        #     self.go_held = True
        #     self.current_state = "armed"
        #     self.arm()

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

    # def arm(self) -> None:
    #     """Set the intake into the "armed" state where it attempts to keep the game object in grip."""
    #     self.m_intake_motor.set(IntakeConstants.armed_speed)

    def bound_shoot(self, arm: ArmSubsystem):
        """Shoot based on arm setpoint."""
        self.shoot(arm.get_setpoint())

    def manual_control(self, speed: float):
        if speed > 0:
            self.intake(False, speed)
        else:
            self.intake(True, speed)

    def periodic(self) -> None:
        SmartDashboard.putBoolean("Intake Sensor", self.sensor.get())
