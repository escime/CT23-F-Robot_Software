from wpimath.controller import PIDController, ProfiledPIDController, \
    ProfiledPIDControllerRadians
import math
import commands2
import commands2.button
import commands2.cmd
from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from pathplannerlib import PathPlanner, PathPlannerTrajectory
from commands.return_wheels import ReturnWheels


def follow_trajectory(traj: PathPlannerTrajectory, first_path: bool, drive: DriveSubsystem) -> \
        commands2.Swerve4ControllerCommand:
    """Return automated command to follow a generated trajectory."""
    theta_controller = ProfiledPIDControllerRadians(AutoConstants.kPThetaController, 0, 0,
                                                    AutoConstants.kThetaControllerConstraints, 0.02)
    theta_controller.enableContinuousInput(-math.pi, math.pi)
    if first_path:
        drive.reset_odometry(traj.getInitialHolonomicPose())
    wpi_traj = traj.asWPILibTrajectory()
    scc = commands2.Swerve4ControllerCommand(wpi_traj,
                                             drive.get_pose,
                                             DriveConstants.m_kinematics,
                                             PIDController(0, 0, 0),
                                             PIDController(0, 0, 0),
                                             theta_controller,
                                             drive.set_module_states,
                                             [drive]
                                             )
    return scc


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
        commands2.cmd.run(leds.rainbow_shift()),
        commands2.WaitCommand(1),
        commands2.ParallelCommandGroup(
            path2,
            commands2.cmd.run(leds.heading_lock(drive.get_heading()))
        ),
        path3
    )


def simple_path(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    # Load a simple path from the pathplanner directory on the roboRIO.
    path = PathPlanner.loadPath("SimplePath", 3, 10, False)
    # Convert path into command for drive subsystem.
    path_command = follow_trajectory(path, True, drive)
    # Return the sequence of auto commands.
    return commands2.SequentialCommandGroup(
        commands2.ParallelRaceGroup(
            path_command,  # Run drive by path command.
            commands2.cmd.run(lambda: leds.fire([255, 0, 0], False), [leds]),  # Set LEDs to fire mode (red)
        ),
        commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])  # On completion, set LEDs to flash
    )


def test_commands(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    return commands2.SequentialCommandGroup(
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.purple_chaser(), [leds]),
            commands2.cmd.runOnce(lambda: drive.set_start_position(90, 10, 10))
        ),
        commands2.ParallelRaceGroup(
            ReturnWheels(drive),  # Set wheels to native zero position.
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 4), [leds]),  # Set LEDs to flash green.
        ),
        commands2.cmd.run(lambda: leds.fire([0, 255, 0], False), [leds])
    )
