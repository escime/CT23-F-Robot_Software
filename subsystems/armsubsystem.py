import commands2
from constants import ArmConstants
from rev import CANSparkMax
from wpilib import SmartDashboard
from subsystems.drivesubsystem import DriveSubsystem


class ArmSubsystem(commands2.SubsystemBase):

    m_arm_motor = CANSparkMax(ArmConstants.masterControlID, CANSparkMax.MotorType.kBrushless)
    pid = m_arm_motor.getPIDController()
    encoder = m_arm_motor.getEncoder()

    setpoints = {"stow": 0,
                 "intake": 10,
                 "shoot_high_front": 6,
                 "shoot_mid_front": 7.7,
                 "shoot_high_back": 2.6,
                 "shoot_mid_back": 1}

    threshold = 1

    current_setpoint = "stow"

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
        self.encoder.setPosition(0)

        self.m_arm_motor.burnFlash()

    def set_position(self, setpoint) -> None:
        """Set target position of the arm."""
        self.pid.setReference(setpoint, CANSparkMax.ControlType.kPosition, 0, 0)

    def get_position(self) -> float:
        """Get the arm's current position."""
        return self.encoder.getPosition()

    def check_completion(self) -> bool:
        """Check if the arm is at its target position."""
        if abs(self.get_position()) - self.threshold < abs(self.setpoints[self.current_setpoint]) < \
                abs(self.get_position()) + self.threshold:
            return True
        else:
            return False

    def set_setpoint(self, setpoint: str) -> None:
        """Set the setpoint for the arm."""
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
        """Get what the arm's current setpoint is."""
        return self.current_setpoint

    def manual_control(self, speed: float):
        self.m_arm_motor.set(speed)
        self.set_setpoint("manual")

    def periodic(self) -> None:
        """Update the dashboard with the arm's current location."""
        SmartDashboard.putNumber("Arm Position", self.get_position())
        SmartDashboard.putString("Arm Setpoint", self.get_setpoint())

    def auto_setpoint(self, drive: DriveSubsystem):
        if 90 < drive.get_heading() % 360 <= 270:
            if self.get_setpoint() != "shoot_high_front":
                self.set_setpoint("shoot_high_front")
        else:
            if self.get_setpoint() != "shoot_mid_back":
                self.set_setpoint("shoot_mid_back")
