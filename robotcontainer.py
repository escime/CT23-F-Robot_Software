import commands2
import commands2.button
import commands2.cmd

from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from subsystems.visionsubsystem import VisionSubsystem
# from subsystems.utilsubsystem import UtilSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.armsubsystem import ArmSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager
import autoplays
from commands.default_leds import DefaultLEDs
# from commands.notifier_led import NotifierLEDs
from commands.debug_mode import DebugMode
from commands.auto_leds import AutoLEDs
from helpers.custom_hid import CustomHID


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # TODO Check if Datalogger slows everything down
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog(), True)
        # Instantiate subsystems using their constructors.
        self.robot_drive = DriveSubsystem()
        self.leds = LEDs(0, 30, 1, 0.03, "GRB")
        self.vision_system = VisionSubsystem(self.robot_drive)
        # self.utilsys = UtilSubsystem()  # Only compatible with REV PDH at this time.
        self.arm = ArmSubsystem()
        self.intake = IntakeSubsystem()

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        # Set the default drive command.
        self.robot_drive.setDefaultCommand(commands2.cmd.run(
            lambda: self.robot_drive.drive(self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                                           self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                                           self.driver_controller_raw.get_axis("RX", 0.06) *
                                           DriveConstants.kMaxAngularSpeed,
                                           True),
            [self.robot_drive]
        ))

        # Set default LED command.
        self.leds.setDefaultCommand(DefaultLEDs(self.leds))

        # Setup for all event-trigger commands.
        self.configureTriggers()

        # Setup autonomous selector on the dashboard.
        self.m_chooser = SendableChooser()
        self.m_chooser.setDefaultOption("No-op", "No-op")
        self.autos = []
        for i in dir(autoplays):
            if str(i[0:5]) == "AUTO_":
                self.autos.append(str(i[5:]))
        for j in self.autos:
            self.m_chooser.addOption(j, j)
        SmartDashboard.putData("Auto Select", self.m_chooser)

        SmartDashboard.putData("Debug Mode On", DebugMode(self.robot_drive, True))
        SmartDashboard.putData("Debug Mode Off", DebugMode(self.robot_drive, False))

    def configureTriggers(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        # Parking Brake.
        commands2.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_lock(), [self.robot_drive]))

        # Slow mode.
        commands2.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_slow(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("RX", 0.06) * DriveConstants.kMaxAngularSpeed,
                True,
                0.5), [self.robot_drive]))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                -90
            ), [self.robot_drive]))
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                90
            ), [self.robot_drive]))
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                0
            ), [self.robot_drive]))
        commands2.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                180
            ), [self.robot_drive]))

        # Reset robot pose to center of the field.
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("Y")).whileTrue(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(), [self.vision_system, self.robot_drive]))

        # Enable Limelight LEDs when B button is held.
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("B")).toggleOnTrue(
            commands2.cmd.run(lambda: self.vision_system.toggle_leds(True), [self.vision_system]))
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("B")).toggleOnFalse(
            commands2.cmd.run(lambda: self.vision_system.toggle_leds(False), [self.vision_system]))

        # When VIEW is held on driver controller, enable auto balancing.
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("VIEW")).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.auto_balance(-1), [self.robot_drive]))

        # When RT is pressed, put the arm and intake to their setpoints for intaking.
        commands2.Trigger(lambda: self.operator_controller_raw.get_trigger("R", 0.3)).whileTrue(
            commands2.ParallelCommandGroup(
                commands2.cmd.run(lambda: self.arm.set_setpoint("intake"), [self.arm]),
                commands2.cmd.run(lambda: self.intake.intake(False, 0.9), [self.intake])))

        # When LB is pressed, the intake spits out anything it has in it without moving the arm
        commands2.Trigger(lambda: self.operator_controller_raw.get_button("LB")).whileTrue(
            commands2.cmd.run(lambda: self.intake.intake(True, 0.5), [self.intake]))

        # When left trigger is pressed, fire at the speed matched to the arm setpoint
        commands2.Trigger(lambda: self.operator_controller_raw.get_trigger("L", 0.5)).whileTrue(
            commands2.cmd.run(lambda: self.intake.bound_shoot(self.arm), [self.intake, self.arm]))

        # Pressing "MENU" enables the directional estimation controller.
        # commands2.Trigger(lambda: self.driver_controller_raw.get_button("MENU")).toggleOnTrue(
        #     commands2.cmd.run(lambda: self.robot_drive.snap_drive(
        #         self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
        #         self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
        #         self.driver_controller_raw.dir_est_ctrl("R")
        #     ))
        # )

        # Reset odometry with vision by using the AUTO PLAY.
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("MENU")).onTrue(
            commands2.SequentialCommandGroup(
                commands2.cmd.runOnce(lambda: self.vision_system.calcs_toggle(), [self.vision_system]),
                autoplays.AUTO_reset_with_vision(self.vision_system, self.robot_drive))
        )

        # When the intake sensor detects a game piece, the robot begins flashing a green alert.
        commands2.Trigger(lambda: self.intake.sensor.get()).whileFalse(
            commands2.cmd.run(lambda: self.leds.flash_color([255, 0, 0], 5), [self.leds]))

        # Force arm to stow.
        commands2.Trigger(lambda: self.operator_controller_raw.get_button("RB")).toggleOnTrue(
            commands2.ParallelCommandGroup(
                commands2.cmd.run(lambda: self.arm.set_setpoint("stow"), [self.arm]),
                commands2.cmd.run(lambda: self.intake.intake(False, 0), [self.intake])))

        # Use the operator controller to set arm setpoints.
        commands2.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.arm.set_setpoint("shoot_high_back"), [self.arm]))
        commands2.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.arm.set_setpoint("shoot_mid_front"), [self.arm]))
        commands2.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.arm.set_setpoint("shoot_mid_back"), [self.arm]))
        commands2.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.arm.set_setpoint("shoot_high_front"), [self.arm]))

        # Configure manual mode for the arm and intake.
        commands2.Trigger(lambda: self.operator_controller_raw.get_button("A")).toggleOnTrue(
            commands2.cmd.run(lambda: self.arm.manual_control(
                self.operator_controller_raw.get_axis("LY", 0.05)), [self.arm]))
        commands2.Trigger(lambda: self.operator_controller_raw.get_button("A")).toggleOnTrue(
            commands2.cmd.run(lambda: self.intake.manual_control(
                self.operator_controller_raw.get_axis("RY", 0.05)), [self.intake]))

        # At the end of auto, enable parking brake.
        commands2.Trigger(lambda: DriverStation.getMatchTime() <= 0.5 and DriverStation.isAutonomous()).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_lock(), [self.robot_drive]))

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "No-op":
            return None
        elif self.m_chooser.getSelected() == "test_commands":
            return autoplays.AUTO_test_commands(self.vision_system, self.robot_drive, self.leds, self.arm, self.intake)
        elif self.m_chooser.getSelected() == "reset_with_vision":
            return autoplays.AUTO_reset_with_vision(self.vision_system, self.robot_drive)
        elif self.m_chooser.getSelected() == "s_m_b":
            return autoplays.AUTO_s_m_b(self.robot_drive, self.leds, self.arm, self.intake)
        elif self.m_chooser.getSelected() == "simple_auto":
            return autoplays.AUTO_simple_auto(self.robot_drive, self.leds, self.arm, self.intake)
        elif self.m_chooser.getSelected() == "s_c_s_m_FLAT":
            return autoplays.AUTO_s_c_s_m_FLAT(self.robot_drive, self.leds, self.arm, self.intake)
        else:
            return None
