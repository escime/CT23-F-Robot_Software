from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.controller import PIDController, ProfiledPIDController
import math
import commands2
import commands2.button
import commands2.cmd
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem
from pathplannerlib import PathPlanner


def path_planner_test(robot_drive) -> commands2.cmd:
    test_path = PathPlanner.loadPath("Test Path", 3, 4, False)
    theta_controller = ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                             AutoConstants.kThetaControllerConstraints)
    theta_controller.enableContinuousInput(-math.pi, math.pi)
    swerve_controller_command = commands2.Swerve4ControllerCommand(test_path,
                                                                   robot_drive.get_pose(),
                                                                   DriveConstants.m_kinematics,
                                                                   PIDController(AutoConstants.kPXController, 0,
                                                                                 0),
                                                                   PIDController(AutoConstants.kPYController, 0,
                                                                                 0),
                                                                   theta_controller,
                                                                   robot_drive.set_module_states,
                                                                   robot_drive)
    robot_drive.reset_odometry(test_path.getInitialPose())

    return swerve_controller_command
