from rev import CANSparkMax
from ctre.sensors import CANCoder
import commands2
import math
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.controller import ProfiledPIDController, PIDController
from subsystems.swervemodule import SwerveModule
from constants import DriveConstants, AutoConstants, ModuleConstants
import navx
from wpilib import SerialPort, SmartDashboard, Field2d
from pathplannerlib import PathPlannerTrajectory


class DriveSubsystem(commands2.SubsystemBase):
    # Creates a new DriveSubsystem
    def __init__(self) -> None:
        super().__init__()
        # Reset odometry @ instantiation.
        self.gyro.reset()
        self.reset_encoders()
        self.m_odometry.update(self.gyro.getRotation2d(),
                               (self.m_FL.get_position(),
                               self.m_FR.get_position(),
                               self.m_BL.get_position(),
                               self.m_BR.get_position()))
        # Setup snap controller for class-wide use.
        self.snap_controller = PIDController(DriveConstants.snap_controller_PID[0],
                                             DriveConstants.snap_controller_PID[1],
                                             DriveConstants.snap_controller_PID[2])

    # Instantiate all swerve modules.
    m_FL = SwerveModule(CANSparkMax(ModuleConstants.fl_drive_id, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(ModuleConstants.fl_turn_id, CANSparkMax.MotorType.kBrushless),
                        CANCoder(ModuleConstants.fl_encoder_id),
                        ModuleConstants.fl_zero_offset,
                        True,
                        True)
    m_FR = SwerveModule(CANSparkMax(ModuleConstants.fr_drive_id, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(ModuleConstants.fr_turn_id, CANSparkMax.MotorType.kBrushless),
                        CANCoder(ModuleConstants.fr_encoder_id),
                        ModuleConstants.fr_zero_offset,
                        True,
                        True)
    m_BL = SwerveModule(CANSparkMax(ModuleConstants.bl_drive_id, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(ModuleConstants.bl_turn_id, CANSparkMax.MotorType.kBrushless),
                        CANCoder(ModuleConstants.bl_encoder_id),
                        ModuleConstants.bl_zero_offset,
                        True,
                        True)
    m_BR = SwerveModule(CANSparkMax(ModuleConstants.br_drive_id, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(ModuleConstants.br_turn_id, CANSparkMax.MotorType.kBrushless),
                        CANCoder(ModuleConstants.br_encoder_id),
                        ModuleConstants.br_zero_offset,
                        True,
                        True)

    # Set initial value of software-tracked position. Should always be zero at startup.
    m_FL_position = m_FL.get_position()
    m_FR_position = m_FR.get_position()
    m_BL_position = m_BL.get_position()
    m_BR_position = m_BR.get_position()

    # Instantiate gyro on Serial Bus. This is a NavX, planned to convert to Pigeon 2.0.
    gyro = navx.AHRS(SerialPort.Port.kUSB)

    # Create pose estimator (replacement for odometry).
    m_odometry = SwerveDrive4PoseEstimator(DriveConstants.m_kinematics, gyro.getRotation2d(),
                                           (m_FL_position, m_FR_position, m_BL_position, m_BR_position),
                                           Pose2d(Translation2d(0, 0), Rotation2d(0)))

    # Create Field2d object to display/track robot position.
    m_field = Field2d()

    def drive(self, x_speed, y_speed, rot, field_relative, teleop) -> None:
        if not teleop:
            if field_relative:
                swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, rot, self.gyro.getRotation2d()))
            else:
                swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(ChassisSpeeds(x_speed,
                                                                                                      y_speed, rot))
        else:
            # TODO deprecate this terrible deadband implementation. Already fixed via CustomHID.
            if abs(x_speed) >= 0.1 or abs(y_speed) >= 0.1 or abs(rot) >= 0.1:
                if field_relative:
                    swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, rot, self.gyro.getRotation2d()))
                else:
                    swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                        ChassisSpeeds(x_speed, y_speed, rot))
            else:
                swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                    ChassisSpeeds(0, 0, 0)
                )

        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.kMaxSpeed)

        self.m_FL.set_desired_state(swerve_module_states[0])
        SmartDashboard.putNumber("FL Target", swerve_module_states[0].angle.degrees())
        SmartDashboard.putNumber("FL Target Speed", swerve_module_states[0].speed)
        self.m_FR.set_desired_state(swerve_module_states[1])
        SmartDashboard.putNumber("FR Target", swerve_module_states[1].angle.degrees())
        SmartDashboard.putNumber("FR Target Speed", swerve_module_states[1].speed)
        self.m_BL.set_desired_state(swerve_module_states[2])
        SmartDashboard.putNumber("BL Target", swerve_module_states[2].angle.degrees())
        SmartDashboard.putNumber("BL Target Speed", swerve_module_states[2].speed)
        self.m_BR.set_desired_state(swerve_module_states[3])
        SmartDashboard.putNumber("BR Target", swerve_module_states[3].angle.degrees())
        SmartDashboard.putNumber("BR Target Speed", swerve_module_states[3].speed)

    def drive_slow(self, x_speed, y_speed, rot, field_relative, teleop, slow: float) -> None:
        self.drive(x_speed * slow, y_speed * slow, rot * slow, field_relative, teleop)

    def drive_lock(self) -> None:
        swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
            ChassisSpeeds(0, 0, 0.01))

        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, DriveConstants.kMaxSpeed)

        self.m_FL.set_desired_state(swerve_module_states[0])
        self.m_FR.set_desired_state(swerve_module_states[1])
        self.m_BL.set_desired_state(swerve_module_states[2])
        self.m_BR.set_desired_state(swerve_module_states[3])

    def periodic(self):
        self.m_odometry.update(self.gyro.getRotation2d(),
                               (self.m_FL.get_position(),
                               self.m_FR.get_position(),
                               self.m_BL.get_position(),
                               self.m_BR.get_position()))
        self.m_field.setRobotPose(self.get_pose())
        SmartDashboard.putData("Field", self.m_field)
        SmartDashboard.putNumber("Robot Heading", self.gyro.getRotation2d().degrees())
        SmartDashboard.putNumber("Robot Pitch", self.gyro.getPitch())
        SmartDashboard.putNumber("FL Angle", self.m_FL.get_state().angle.degrees())
        SmartDashboard.putNumber("FL Speed", self.m_FL.get_state().speed)
        SmartDashboard.putNumber("FR Angle", self.m_FR.get_state().angle.degrees())
        SmartDashboard.putNumber("FR Speed", self.m_FR.get_state().speed)
        SmartDashboard.putNumber("BL Angle", self.m_BL.get_state().angle.degrees())
        SmartDashboard.putNumber("BL Speed", self.m_BL.get_state().speed)
        SmartDashboard.putNumber("BR Angle", self.m_BR.get_state().angle.degrees())
        SmartDashboard.putNumber("BR Speed", self.m_BR.get_state().speed)
        SmartDashboard.putString("Current Command", str(self.getCurrentCommand()))

    def get_pose(self):
        return self.m_odometry.getEstimatedPosition()

    def add_vision(self, pose: Pose2d, timestamp: float):
        self.m_odometry.addVisionMeasurement(pose, timestamp)

    def reset_odometry(self, pose: Pose2d):
        self.m_odometry.resetPosition(self.gyro.getRotation2d(), (self.m_FL_position, self.m_FR_position,
                                      self.m_BL_position, self.m_BR_position), pose)

    def set_module_states(self, desired_states):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, DriveConstants.kMaxSpeed)
        self.m_FL.set_desired_state(desired_states[0])
        self.m_FR.set_desired_state(desired_states[1])
        self.m_BL.set_desired_state(desired_states[2])
        self.m_BR.set_desired_state(desired_states[3])

    def reset_encoders(self):
        self.m_FL.reset_encoders()
        self.m_FR.reset_encoders()
        self.m_BL.reset_encoders()
        self.m_BR.reset_encoders()

    def zero_heading(self):
        self.gyro.reset()

    def get_heading(self):
        return self.gyro.getYaw()

    def get_turn_rate(self):
        if DriveConstants.kGyroReversed:
            return self.gyro.getRate() * -1.0
        else:
            return self.gyro.getRate()

    def follow_trajectory(self, traj: PathPlannerTrajectory, first_path: bool) -> commands2.Swerve4ControllerCommand:
        theta_controller = ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                                 AutoConstants.kThetaControllerConstraints)
        theta_controller.enableContinuousInput(-math.pi, math.pi)
        if first_path:
            self.reset_odometry(traj.getInitialHolonomicPose())
        swerve_controller_command = commands2.Swerve4ControllerCommand(traj,
                                                                       self.get_pose(),
                                                                       DriveConstants.m_kinematics,
                                                                       PIDController(AutoConstants.kPXController, 0,
                                                                                     0),
                                                                       PIDController(AutoConstants.kPYController, 0,
                                                                                     0),
                                                                       theta_controller,
                                                                       self.set_module_states,
                                                                       self)
        return swerve_controller_command

    def snap_drive(self, x_speed: float, y_speed: float, heading_target: float):
        """Calculate and implement the PID controller for rotating to and maintaining a target heading."""
        if self.gyro.getRotation2d().degrees() >= 0:
            current_heading = self.gyro.getRotation2d().degrees() % 360
        else:
            current_heading = self.gyro.getRotation2d().degrees() % -360
        if abs(current_heading - heading_target) > 180:
            heading_target = -1 * heading_target
        rotate_output = self.snap_controller.calculate(current_heading, heading_target)
        self.drive(x_speed, y_speed, -rotate_output, True, False)
