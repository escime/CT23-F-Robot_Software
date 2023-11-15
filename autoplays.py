from wpimath.controller import PIDController, ProfiledPIDControllerRadians
import math
import commands2
import commands2.button
import commands2.cmd
from constants import AutoConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from subsystems.armsubsystem import ArmSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.visionsubsystem import VisionSubsystem
from commands.return_wheels import ReturnWheels
from commands.set_arm import SetArm
from commands.shoot import Shoot
from commands.intake import Intake
from commands.turn import Turn
from commands.drive_to_CS import DriveToCS
from commands.vision_estimate import VisionEstimate
from commands.swerve_controller import SwerveControllerCommand
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory


# def follow_trajectory(traj: Trajectory, first_path: bool, drive: DriveSubsystem) -> \
#         commands2.Swerve4ControllerCommand:
def follow_trajectory(traj: Trajectory, first_path: bool, drive: DriveSubsystem) -> \
        SwerveControllerCommand:
    """Return automated command to follow a generated trajectory."""
    theta_controller = ProfiledPIDControllerRadians(AutoConstants.kPThetaController, 0, 0,
                                                    AutoConstants.kThetaControllerConstraints, 0.02)
    theta_controller.enableContinuousInput(-math.pi, math.pi)
    if first_path:
        drive.reset_odometry(traj.initialPose())
    # wpi_traj = traj
    # scc = commands2.Swerve4ControllerCommand(wpi_traj,
    #                                          drive.get_pose,
    #                                          DriveConstants.m_kinematics,
    #                                          PIDController(AutoConstants.kPXController, 0, 0),
    #                                          PIDController(AutoConstants.kPYController, 0, 0),
    #                                          theta_controller,
    #                                          drive.set_module_states,
    #                                          [drive]
    #                                          )
    scc = SwerveControllerCommand(traj,
                                  DriveConstants.m_kinematics,
                                  PIDController(AutoConstants.kPXController, 0, 0),
                                  PIDController(AutoConstants.kPYController, 0, 0),
                                  theta_controller,
                                  traj.states()[-1].pose.rotation(),
                                  drive)
    return scc


def generate_wpi_trajectory(start: Pose2d, waypoints: [Translation2d], end: Pose2d, vmax: float, amax: float) \
        -> Trajectory:
    """Generates a wpi trajectory from a starting pose, list of waypoints, ending pose, and constraints."""
    return TrajectoryGenerator.generateTrajectory(start, waypoints, end, TrajectoryConfig(vmax, amax))


def map_to_red(x: float, y: float, theta: float, mapp: bool) -> [float, float, float]:
    """Maps a pose from the blue side of the field to the red side of the field."""
    if mapp:
        return Pose2d(16.45 - x, y, Rotation2d().fromDegrees(-1 * theta))
    else:
        return Pose2d(x, y, Rotation2d().fromDegrees(theta))


def map_to_red_trans(x: float, y: float, mapp: bool) -> [float, float]:
    """Maps a translation from the blue side of the field to the red side of the field."""
    if mapp:
        return Translation2d(16.45 - x, y)
    else:
        return Translation2d(x, y)


def gen_and_run(start: Pose2d, waypoints: [Translation2d], end: Pose2d, vmax: float, amax: float, first_path: bool,
                drive: DriveSubsystem) -> commands2.SequentialCommandGroup:
    """Generates a path following command based on the constraints for a path given."""
    return commands2.SequentialCommandGroup(
        follow_trajectory(generate_wpi_trajectory(start, waypoints, end, vmax, amax), first_path, drive),
        commands2.cmd.run(lambda: drive.drive(0, 0, 0, False), drive)
    )


def AUTO_test_commands(vision: VisionSubsystem, drive: DriveSubsystem, leds: LEDs,
                       arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    """For testing commands."""
    # if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
    #     inverted = True
    # else:
    #     inverted = False
    # path1 = generate_wpi_trajectory(Pose2d(1.75, 4.43, 0), [Translation2d(3.19, 4.94)], Pose2d(6.73, 4.62, 5.53), 4,3)
    # path_command_1 = follow_trajectory(path1, True, drive)
    # return commands2.SequentialCommandGroup(
    #     commands2.ParallelRaceGroup(
    #         path_command_1,
    #         commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), [leds])
    #     )
    # )
    return commands2.SequentialCommandGroup(
        VisionEstimate(vision, drive)
    )


def AUTO_reset_with_vision(vision: VisionSubsystem, drive: DriveSubsystem) -> commands2.SequentialCommandGroup:
    return commands2.SequentialCommandGroup(
        VisionEstimate(vision, drive)
    )


def AUTO_s_m_b(drive: DriveSubsystem, leds: LEDs,
               arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    """Score -> Mobility -> Balance."""
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        inverted = True
    else:
        inverted = False
    path_command_1 = gen_and_run(map_to_red(1.74, 2.73, 0, inverted),
                                 [map_to_red_trans(4.14, 2.73, inverted)],
                                 map_to_red(7.00, 2.73, 0, inverted), 2, 2, True, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        SetArm(arm, "stow"),
        Intake(intake, False, 0),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), leds)
        ),
        commands2.WaitCommand(1),
        commands2.ParallelRaceGroup(
            Turn(drive, 0),
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 5), leds)),
        commands2.ParallelRaceGroup(
            DriveToCS(drive, 0),
            commands2.cmd.run(lambda: leds.flash_color([0, 255, 0], 5), leds)),
        commands2.ParallelRaceGroup(
            commands2.cmd.run(lambda: leds.rainbow_shift(), leds),
            commands2.cmd.run(lambda: drive.auto_balance(-1))
        )
    )


def AUTO_simple_auto(drive: DriveSubsystem, leds: LEDs,
                     arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    """Score -> Mobility."""
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        inverted = True
    else:
        inverted = False
    path_command_1 = gen_and_run(map_to_red(1.81, 5.05, 0, inverted),
                                 [],
                                 map_to_red(6.50, 5.05, 0, inverted), 2, 2, True, drive)
    return commands2.SequentialCommandGroup(
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        SetArm(arm, "stow"),
        Intake(intake, False, 0),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), leds)
        )
    )


def AUTO_s_c_s_m_FLAT(drive: DriveSubsystem, leds: LEDs,
                      arm: ArmSubsystem, intake: IntakeSubsystem) -> commands2.SequentialCommandGroup:
    """Score -> Collect -> Score -> Mobility | Flat Side Auto"""
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        inverted = True
    else:
        inverted = False
    path1 = generate_wpi_trajectory(map_to_red(1.75, 4.43, 0, inverted),
                                    [map_to_red_trans(3.19, 4.94, inverted)],
                                    map_to_red(6.73, 4.62, 5.53, inverted), 4, 3)
    path_command_1 = follow_trajectory(path1, True, drive)
    return commands2.SequentialCommandGroup(
        ReturnWheels(drive),
        SetArm(arm, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_high_back"),
        commands2.WaitCommand(0.5),
        SetArm(arm, "shoot_mid_front"),
        Intake(intake, False, 1),
        commands2.WaitCommand(1),
        commands2.ParallelRaceGroup(
            path_command_1,
            commands2.cmd.run(lambda: leds.flash_color([255, 0, 0], 2), leds)
        ),
        ReturnWheels(drive),
        Intake(intake, False, 0),
        SetArm(arm, "shoot_mid_back"),
        commands2.ParallelRaceGroup(
            gen_and_run(map_to_red(6.73, 4.62, 5.53, inverted),
                        [map_to_red_trans(3.19, 4.94, inverted)],
                        map_to_red(2.3, 4.6, 1, inverted), 4, 3, False, drive),
            commands2.cmd.run(lambda: leds.flash_color([0, 255, 0], 2), leds)
        ),
        ReturnWheels(drive),
        SetArm(arm, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        Shoot(intake, "shoot_mid_back"),
        commands2.WaitCommand(0.5),
        SetArm(arm, "stow"),
        Intake(intake, False, 0),
        ReturnWheels(drive),
        commands2.ParallelRaceGroup(
            gen_and_run(map_to_red(2.3, 4.6, 1, inverted), [], map_to_red(6.73, 4.62, 5.53, inverted), 4, 3, False,
                        drive),
            commands2.cmd.run(lambda: leds.flash_color([0, 0, 255], 2), leds)
        )
    )
