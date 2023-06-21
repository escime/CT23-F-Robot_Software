import commands2
from constants import ArmConstants
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile
from rev import CANSparkMax


class ArmSubsystem(commands2.ProfiledPIDSubsystem):

    m_arm_motor = CANSparkMax(ArmConstants.masterControlID, CANSparkMax.MotorType.kBrushless)
    f_arm_motor = CANSparkMax(ArmConstants.followerControlID, CANSparkMax.MotorType.kBrushless)
    f_arm_motor.follow(m_arm_motor, True)

    encoder = m_arm_motor.getEncoder()
    encoder.setVelocityConversionFactor(ArmConstants.velocityConversion)
    encoder.setPositionConversionFactor(ArmConstants.positionConversion)

    feedforward = ArmFeedforward(ArmConstants.kS,
                                 ArmConstants.kG,
                                 ArmConstants.kV,
                                 ArmConstants.kA)

    def __init__(self) -> None:
        super().__init__(ProfiledPIDController(ArmConstants.kP,
                                               0,
                                               0,
                                               TrapezoidProfile(ArmConstants.kMaxVelRadPerSec,
                                                                ArmConstants.kMaxAclRadPerSecSquared)),
                         initialPosition=self.encoder.getPosition())