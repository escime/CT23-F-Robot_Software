import commands2
from constants import ArmConstants
from rev import CANSparkMax


class ArmSubsystem(commands2.SubsystemBase):

    m_arm_motor = CANSparkMax(ArmConstants.masterControlID, CANSparkMax.MotorType.kBrushless)
    pid = m_arm_motor.getPIDController()

    current_setpoint = "startup"

    def __init__(self) -> None:
        super().__init__()
        self.pid.setP(ArmConstants.kP)
        self.pid.setI(ArmConstants.kI)
        self.pid.setD(ArmConstants.kD)
        self.pid.setFF(ArmConstants.kFF)
        self.pid.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput)
        self.pid.setSmartMotionMaxVelocity(ArmConstants.maxVel, 1)
        self.pid.setSmartMotionMinOutputVelocity(ArmConstants.minVel, 1)
        self.pid.setSmartMotionMaxAccel(ArmConstants.maxAcc, 1)
        self.pid.setSmartMotionAllowedClosedLoopError(ArmConstants.allowedErr, 1)

        self.m_arm_motor.burnFlash()

    def set_position(self, setpoint) -> None:
        self.pid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion)

    def set_setpoint(self, setpoint: str) -> None:
        invalid = False
        if setpoint == "stow":
            self.set_position(0)
        elif setpoint == "intake":
            self.set_position(0)
        elif setpoint == "shoot_high_front":
            self.set_position(0)
        elif setpoint == "shoot_mid_front":
            self.set_position(0)
        elif setpoint == "shoot_high_back":
            self.set_position(0)
        elif setpoint == "shoot_mid_back":
            self.set_position(0)
        else:
            print("Setpoint not recognized.")
            invalid = True
        if not invalid:
            self.current_setpoint = setpoint

    def get_setpoint(self) -> str:
        return self.current_setpoint
