# import wpilib
from rev import CANSparkMax
from ctre.sensors import CANCoder
import commands2
import math
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.controller import ProfiledPIDController, PIDController
from subsystems.swervemodule import SwerveModule
from constants import DriveConstants, AutoConstants
import navx
from wpilib import SerialPort
from pathplannerlib import PathPlannerTrajectory


class DriveSubsystem(commands2.SubsystemBase):
    # Creates a new DriveSubsystem
    def __init__(self) -> None:
        super().__init__()
        self.gyro.reset()
        self.reset_encoders()
        self.m_odometry.update(self.gyro.getRotation2d(),
                               (self.m_FL.get_position(),
                               self.m_FR.get_position(),
                               self.m_BL.get_position(),
                               self.m_BR.get_position()))

    m_FL = SwerveModule(CANSparkMax(10, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(11, CANSparkMax.MotorType.kBrushless),
                        CANCoder(12),
                        -78.75,
                        True,
                        True)
    m_FR = SwerveModule(CANSparkMax(13, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(14, CANSparkMax.MotorType.kBrushless),
                        CANCoder(15),
                        -175.341797,
                        True,
                        True)
    m_BL = SwerveModule(CANSparkMax(16, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(17, CANSparkMax.MotorType.kBrushless),
                        CANCoder(18),
                        -293.554688,
                        True,
                        True)
    m_BR = SwerveModule(CANSparkMax(19, CANSparkMax.MotorType.kBrushless),
                        CANSparkMax(20, CANSparkMax.MotorType.kBrushless),
                        CANCoder(21),
                        -250.136719,
                        True,
                        True)
    # CANCoder(21) replaces CANSparkMax(20, CANSparkMax.MotorType.kBrushless).getEncoder()

    m_FL_position = m_FL.get_position()
    m_FR_position = m_FR.get_position()
    m_BL_position = m_BL.get_position()
    m_BR_position = m_BR.get_position()

    gyro = navx.AHRS(SerialPort.Port.kUSB)

    m_odometry = SwerveDrive4PoseEstimator(DriveConstants.m_kinematics, gyro.getRotation2d(),
                                           (m_FL_position, m_FR_position, m_BL_position, m_BR_position),
                                           Pose2d(Translation2d(0, 0), Rotation2d(0)))

    def drive(self, x_speed, y_speed, rot, field_relative, teleop) -> None:
        if not teleop:
            if field_relative:
                swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, rot, self.gyro.getRotation2d()))
            else:
                swerve_module_states = DriveConstants.m_kinematics.toSwerveModuleStates(ChassisSpeeds(x_speed,
                                                                                                      y_speed, rot))
        else:
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
        self.m_FR.set_desired_state(swerve_module_states[1])
        self.m_BL.set_desired_state(swerve_module_states[2])
        self.m_BR.set_desired_state(swerve_module_states[3])

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
        # print("FL: " + str(self.m_FL.get_position()))
        # print("FR: " + str(self.m_FR.get_position()))
        # print("BL: " + str(self.m_BL.get_position()))
        # print("BR: " + str(self.m_BR.get_position()))
        # SmartDashboard.putData("Robot Heading", self.gyro.getRotation2d().degrees())

    def get_pose(self):
        return self.m_odometry.getEstimatedPosition()

    def reset_odometry(self, pose):
        self.m_odometry.resetPosition(self.gyro.getRotation2d(), pose, (self.m_FL_position, self.m_FR_position,
                                      self.m_BL_position, self.m_BR_position))

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
