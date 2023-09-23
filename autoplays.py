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
from commands.turn import Turn
from commands.drive_to_CS import DriveToCS
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d


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


def AUTO_test_commands(drive: DriveSubsystem, leds: LEDs,
                       arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    paths = ["A-B"]
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        inverted = True
    else:
        inverted = False
    path1 = PathPlanner.loadPath(paths[0], 4, 2, inverted)
    path_command_1 = follow_trajectory(path1, True, drive)
    return commands2.SequentialCommandGroup(
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
        )
    )


def s_c_s_NO_BUMP(drive: DriveSubsystem, leds: LEDs, arm: ArmSubsystem,
                     intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        invert = True
    else:
        invert = False
    path1 = PathPlanner.loadPath("6B-A", 4, 3, invert)
    path2 = PathPlanner.loadPath("A-6C", 4, 3, invert)
    path_command_1 = follow_trajectory(path1, True, drive)
    path_command_2 = follow_trajectory(path2, False, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Intake(intake, False, 0.8),
        SetArm(arm, "intake"),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
        ),
        commands2.cmd.runOnce(lambda: drive.drive(0, 0, 0, True), [drive]),
        commands2.WaitCommand(1),
        Intake(intake, False, 0),
        SetArm(arm, "stow"),
        commands2.ParallelRaceGroup(
            path_command_2,
            commands2.cmd.run(lambda: leds.flash_color([0, 0, 255], 2), [leds])
        ),
        commands2.cmd.runOnce(lambda: drive.drive(0, 0, 0, True), [drive]),
        SetArm(arm, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Intake(intake, False, 0),
        SetArm(arm, "stow")
    )


def s_BUMP(drive: DriveSubsystem, leds: LEDs, arm: ArmSubsystem,
                     intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        invert = True
    else:
        invert = False
    path1 = PathPlanner.loadPath("8B-D", 4, 3, invert)
    path_command_1 = follow_trajectory(path1, True, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Intake(intake, False, 0),
        SetArm(arm, "stow"),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds]),
        )
    )


def RED_s_BUMP(drive: DriveSubsystem, leds: LEDs, arm: ArmSubsystem,
                     intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        invert = True
    else:
        invert = False
    path1 = PathPlanner.loadPath("8B-D", 4, 3, invert)
    path_command_1 = follow_trajectory(path1, True, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Intake(intake, False, 0),
        SetArm(arm, "stow"),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds]),
        )
    )


def AUTO_s_b_b(drive: DriveSubsystem, leds: LEDs,
               arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        inverted = True
        path = "RED_7B-CSB"
    else:
        inverted = False
        path = "7B-CSB"
    path1 = PathPlanner.loadPath(path, 2, 2, inverted)
    path_command_1 = follow_trajectory(path1, True, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        SetArm(arm, "stow"),
        Intake(intake, False, 0),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
        ),
        commands2.WaitCommand(1),
        commands2.ParallelRaceGroup(
            Turn(drive, 0),
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 5), [leds])),
        commands2.ParallelRaceGroup(
            DriveToCS(drive, 0),
            commands2.cmd.run(lambda: leds.flash_color([0, 255, 0], 5), [leds])),
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.rainbow_shift(), [leds]),
            commands2.cmd.run(lambda: drive.auto_balance(-1))
        )
    )


def AUTO_simple_auto(drive: DriveSubsystem, leds: LEDs,
                arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        path = "SimpleAutoRed"
        inverted = True
    else:
        path = "SimpleAutoBlue"
        inverted = False
    path1 = PathPlanner.loadPath(path, 2, 2, inverted)
    path_command_1 = follow_trajectory(path1, True, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        SetArm(arm, "stow"),
        Intake(intake, False, 0),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
        )
    )
