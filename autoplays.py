from wpimath.controller import PIDController, ProfiledPIDController, \
    ProfiledPIDControllerRadians
import math
import commands2
import commands2.button
import commands2.cmd
from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from subsystems.armsubsystem import ArmSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from pathplannerlib import PathPlanner, PathPlannerTrajectory
from commands.return_wheels import ReturnWheels
from commands.set_arm import SetArm
from commands.shoot import Shoot
from commands.intake import Intake
from wpilib import DriverStation


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
                                             PIDController(AutoConstants.kPXController, 0, 0),
                                             PIDController(AutoConstants.kPYController, 0, 0),
                                             theta_controller,
                                             drive.set_module_states,
                                             [drive]
                                             )
    return scc


def AUTO_path_planner_test(robot_drive: DriveSubsystem) -> commands2.cmd:
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


def AUTO_path_points_test(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
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


def AUTO_simple_path(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    # Load a simple path from the pathplanner directory on the roboRIO.
    path = PathPlanner.loadPath("SimplePath", 4, 3, False)
    # Convert path into command for drive subsystem.
    path_command = follow_trajectory(path, True, drive)
    # Return the sequence of auto commands.
    return commands2.SequentialCommandGroup(
        commands2.ParallelRaceGroup(
            path_command,  # Run drive by path command.
            commands2.cmd.run(lambda: leds.fire([255, 0, 0], False), [leds]),  # Set LEDs to fire mode (red)
        ),
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.purple_chaser(), [leds]),  # On completion, set LEDs to default
            ReturnWheels(drive)
        )
    )


def AUTO_test_commands(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    return commands2.SequentialCommandGroup(
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.purple_chaser(), [leds]),
            commands2.cmd.runOnce(lambda: drive.set_start_position(90, 4, 2))
        ),
        commands2.ParallelRaceGroup(
            ReturnWheels(drive),  # Set wheels to native zero position.
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 4), [leds]),  # Set LEDs to flash green.
        ),
        commands2.cmd.run(lambda: leds.fire([0, 255, 0], False), [leds])
    )


def AUTO_quik_auto(drive: DriveSubsystem, leds: LEDs) -> commands2.SequentialCommandGroup:
    # Load a simple path from the pathplanner directory on the roboRIO.
    path = PathPlanner.loadPath("QuikAuto", 4, 3, False)
    # Convert path into command for drive subsystem.
    path_command = follow_trajectory(path, True, drive)
    # Return the sequence of auto commands.
    return commands2.SequentialCommandGroup(
        commands2.ParallelRaceGroup(
            path_command,  # Run drive by path command.
            commands2.cmd.run(lambda: leds.fire([255, 0, 0], False), [leds]),  # Set LEDs to fire mode (red)
        ),
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.purple_chaser(), [leds]),  # On completion, set LEDs to default
            ReturnWheels(drive)
        )
    )


def AUTO_s_c_s_c_b_s(drive: DriveSubsystem, leds: LEDs, arm: ArmSubsystem,
                     intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        invert = True
    else:
        invert = False
    path1 = PathPlanner.loadPath("6B-A", 4, 3, invert)
    path2 = PathPlanner.loadPath("A-B", 4, 3, invert)
    path3 = PathPlanner.loadPath("B-CSB", 4, 3, invert)
    path_command_1 = follow_trajectory(path1, True, drive)
    path_command_2 = follow_trajectory(path2, False, drive)
    path_command_3 = follow_trajectory(path3, False, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        SetArm(arm, "intake"),
        Intake(intake, False, 1),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
        ),
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        SetArm(arm, "intake"),
        Intake(intake, False, 1),
        commands2.ParallelRaceGroup(
            path_command_2,
            commands2.cmd.run(lambda: leds.flash_color([0, 255, 0], 2), [leds])
        ),
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        SetArm(arm, "stow"),
        Intake(intake, False, 0),
        commands2.ParallelRaceGroup(
            path_command_3,
            commands2.cmd.run(lambda: leds.flash_color([0, 0, 255], 2), [leds])
        ),
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.rainbow_shift(), [leds]),
            commands2.cmd.run(lambda: drive.auto_balance(-1))
        )
    )


def AUTO_s_b_b(drive: DriveSubsystem, leds: LEDs,
               arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        invert = True
    else:
        invert = False
    path1 = PathPlanner.loadPath("7B-CSB", 4, 3, invert)
    path_command_1 = follow_trajectory(path1, True, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.25),
        SetArm(arm, "intake"),
        Intake(intake, False, 0.25),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
        ),
        commands2.cmd.run(lambda: drive.snap_drive(0, 0, 180)),  # May need to make a legit turn-to-angle command
        commands2.WaitCommand(1),
        commands2.cmd.run(lambda: drive.snap_drive(0, -0.5 * DriveConstants.kMaxSpeed, 180)),  # probs no worky
        commands2.WaitCommand(0.5),
        SetArm(arm, "stow"),
        Intake(intake, False, 0),
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.rainbow_shift(), [leds]),
            commands2.cmd.run(lambda: drive.auto_balance(-1))
        )
    )
