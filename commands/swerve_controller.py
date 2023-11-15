from commands2 import Command

from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import Trajectory
from subsystems.drivesubsystem import DriveSubsystem
from wpilib import Timer


class SwerveControllerCommand(Command):
    m_timer = Timer()
    m_trajectory: Trajectory
    m_kinematics: SwerveDrive4Kinematics
    m_controller: HolonomicDriveController
    m_desired_rotation: Rotation2d
    m_drive: DriveSubsystem

    def __init__(self, trajectory: Trajectory, kinematics: SwerveDrive4Kinematics,
                 x_controller: PIDController, y_controller: PIDController, theta_controller: ProfiledPIDController,
                 desired_rotation: Rotation2d, subsystem: DriveSubsystem):
        super().__init__()
        self.m_trajectory = trajectory
        self.m_kinematics = kinematics
        self.m_controller = HolonomicDriveController(x_controller, y_controller, theta_controller)
        self.m_desired_rotation = desired_rotation
        self.m_drive = subsystem
        self.hasRequirement(subsystem)

    def initialize(self) -> None:
        self.m_timer.restart()

    def execute(self) -> None:
        current_time = self.m_timer.get()
        desired_state = self.m_trajectory.sample(current_time)
        target_chassis_speeds = self.m_controller.calculate(self.m_drive.get_pose(), desired_state,
                                                            self.m_desired_rotation)
        target_module_states = self.m_kinematics.toSwerveModuleStates(target_chassis_speeds)

        self.m_drive.set_module_states(target_module_states)

    def end(self, interrupted: bool) -> None:
        self.m_timer.stop()

    def isFinished(self) -> bool:
        return self.m_timer.hasElapsed(self.m_trajectory.totalTime())
