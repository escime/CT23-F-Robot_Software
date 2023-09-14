import commands2
from constants import ArmConstants
from rev import CANSparkMax
from wpilib import SmartDashboard


class ArmSubsystem(commands2.SubsystemBase):

    m_arm_motor = CANSparkMax(ArmConstants.masterControlID, CANSparkMax.MotorType.kBrushless)
    pid = m_arm_motor.getPIDController()
    encoder = m_arm_motor.getEncoder()

    setpoints = {"startup": 0,
                 "stow": 0,
                 "intake": 0,
                 "shoot_high_front": 0,
                 "shoot_mid_front": 0,
                 "shoot_high_back": 0,
                 "shoot_mid_back": 0}

    threshold = 20

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
        self.pid.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion, 1, 0)

    def get_position(self) -> float:
        return self.encoder.getPosition()

    def check_completion(self) -> bool:
        if abs(self.get_position()) - self.threshold < abs(self.setpoints[self.current_setpoint]) < \
                abs(self.get_position()) + self.threshold:
            return True
        else:
            return False

    def set_setpoint(self, setpoint: str) -> None:
        invalid = False
        if setpoint == "stow":
            self.set_position(self.setpoints["stow"])
        elif setpoint == "intake":
            self.set_position(self.setpoints["intake"])
        elif setpoint == "shoot_high_front":
            self.set_position(self.setpoints["shoot_high_front"])
        elif setpoint == "shoot_mid_front":
            self.set_position(self.setpoints["shoot_mid_front"])
        elif setpoint == "shoot_high_back":
            self.set_position(self.setpoints["shoot_high_back"])
        elif setpoint == "shoot_mid_back":
            self.set_position(self.setpoints["shoot_mid_back"])
        else:
            print("Setpoint not recognized.")
            invalid = True
        if not invalid:
            self.current_setpoint = setpoint

    def get_setpoint(self) -> str:
        return self.current_setpoint

    def periodic(self) -> None:
        SmartDashboard.putNumber("Arm Position", self.get_position())
