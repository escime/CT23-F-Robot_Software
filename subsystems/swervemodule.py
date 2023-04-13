from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from rev import CANSparkMax
from ctre.sensors import CANCoder, AbsoluteSensorRange
from constants import DriveConstants
import math



class SwerveModule:
    driveMotor: CANSparkMax
    rotateMotor: CANSparkMax
    encoder: CANCoder

    # cfg: ModuleConfig

    def __init__(self, dm, rm, enc, mod_offset, turn_invert, drive_invert):
        self.driveMotor = dm
        self.rotateMotor = rm
        # self.encoder = rm.getEncoder()
        self.encoder = enc
        # self.absolute_check = rm.getAbsoluteEncoder()

        self.drive_encoder = self.driveMotor.getEncoder()
        self.drive_encoder.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
        self.drive_encoder.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)
        self.encoder.setPositionToAbsolute()
        self.encoder.configMagnetOffset(mod_offset)
        self.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)

        self.driveMotor.setInverted(drive_invert)
        self.rotateMotor.setInverted(turn_invert)

        self._requested_turn = 0
        self._requested_speed = 0

        self._drive_pid_controller = PIDController(0.7, 0, 0)  # Put in module constants later
        self._rotate_pid_controller = PIDController(0.7, 0, 0)
        self._rotate_pid_controller.enableContinuousInput(-math.pi, math.pi)

        # self._drive_pid_controller.setTolerance(0.05, 0.05)

        # self.rotate_feed_forward = SimpleMotorFeedforwardMeters(0.14165, 2.8026, 0.034594)  adjust these numbers later
        self.drive_feed_forward = SimpleMotorFeedforwardMeters(0.22/12, 1.0/12, 0.23/12)  # adjust these numbers later

        rm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        dm.setIdleMode(CANSparkMax.IdleMode.kBrake)
        rm.burnFlash()
        dm.burnFlash()

    def get_state(self):
        return SwerveModuleState(self.drive_encoder.getVelocity(), Rotation2d(math.radians(self.encoder.getAbsolutePosition())))

    def get_position(self):
        return SwerveModulePosition(self.drive_encoder.getPosition(), Rotation2d(math.radians(self.encoder.getAbsolutePosition())))

    def set_desired_state(self, desired_state):
        if abs(desired_state.angle.radians() - math.radians(self.encoder.getAbsolutePosition())) >= math.radians(40):
            state = self.optimize_module(desired_state)
            # state = SwerveModuleState.optimize(desired_state, Rotation2d(math.radians(self.encoder.getAbsolutePosition())))
        else:
            state = desired_state
        drive_output = self._drive_pid_controller.calculate(self.drive_encoder.getVelocity(), state.speed)
        drive_ff = self.drive_feed_forward.calculate(state.speed)
        rotate_output = self._rotate_pid_controller.calculate(math.radians(self.encoder.getAbsolutePosition()), state.angle.radians())
        # rotate_ff = self.drive_feed_forward.calculate(self._rotate_pid_controller.getSetpoint())
        # above line may be an issue. missing velocity???
        # self._rotate_pid_controller.setReference(state.angle.radians(), CANSparkMax.ControlType.kPosition)
        # CANSparkMax.getPIDController().setReference()

        self.driveMotor.setVoltage(drive_output + drive_ff)
        self.rotateMotor.setVoltage(rotate_output)

    def reset_encoders(self):
        # self.encoder.setPosition(0)
        self.drive_encoder.setPosition(0)

    def optimize_module(self, desired_state: SwerveModuleState) -> SwerveModuleState:
        inverted = False
        desired_degrees = desired_state.angle.degrees()  # 360.0
        if desired_degrees < 0.0:
            desired_degrees += 360.0

        current_degrees = self.encoder.getAbsolutePosition()
        current_mod = current_degrees % 360.0
        if current_mod < 0.0:
            current_mod += 360

        if 90.0 < abs(current_mod - desired_degrees) <= 270.0:
            inverted = True
            desired_degrees -= 180.0

        delta_angle = desired_degrees - current_mod
        if delta_angle < 0.0:
            delta_angle += 360.0

        ccw_angle = delta_angle
        cw_angle = delta_angle - 360.0

        if abs(ccw_angle) < abs(cw_angle):
            desired_degrees = ccw_angle
        else:
            desired_degrees = cw_angle

        magnitude = desired_state.speed

        if inverted:
            magnitude = magnitude * -1

        desired_degrees += current_degrees

        return SwerveModuleState(magnitude, Rotation2d.fromDegrees(desired_degrees))
