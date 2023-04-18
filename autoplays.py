# from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
# from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.controller import PIDController, ProfiledPIDController
import math
import commands2
import commands2.button
import commands2.cmd
from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from pathplannerlib import PathPlanner


def path_planner_test(robot_drive: DriveSubsystem) -> commands2.cmd:
    test_path = PathPlanner.loadPath("Test Path", 3, 4, False)
    theta_controller = ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                             AutoConstants.kThetaControllerConstraints)
    theta_controller.enableContinuousInput(-math.pi, math.pi)
    robot_drive.reset_odometry(test_path.getInitialHolonomicPose())
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
    return swerve_controller_command


def path_points_test(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    path_group = PathPlanner.loadPathGroup("Path1", 3, 4, False)
    path_group[0] = path_group[0].asWPILibTrajectory()
    path_group[1] = path_group[1].asWPILibTrajectory()
    path_group[2] = path_group[2].asWPILibTrajectory()
    path1 = drive.follow_trajectory(path_group[0], True)
    path2 = drive.follow_trajectory(path_group[1], False)
    path3 = drive.follow_trajectory(path_group[2], False)
    return commands2.SequentialCommandGroup(
        path1,
        commands2.cmd.run(leds.rainbow_shift(True)),
        commands2.WaitCommand(1),
        commands2.ParallelCommandGroup(
            path2,
            commands2.cmd.run(leds.heading_lock(drive.get_heading()))
        ),
        path3
    )
