import commands2
import commands2.button
import commands2.cmd
# import wpilib

from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from subsystems.visionsubsystem import VisionSubsystem
# from subsystems.utilsubsystem import UtilSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.armsubsystem import ArmSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation
import autoplays
from commands.default_leds import DefaultLEDs
from commands.notifier_led import NotifierLEDs
from commands.debug_mode import DebugMode
from helpers.custom_hid import CustomHID


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Instantiate subsystems using their constructors.
        self.robot_drive = DriveSubsystem()
        self.leds = LEDs(0, 50, 1, 0.03, "GRB")
        self.vision_system = VisionSubsystem(self.robot_drive)
        # self.utilsys = UtilSubsystem()
        self.arm = ArmSubsystem()
        self.intake = IntakeSubsystem()

        self.vision_system.setDefaultCommand(commands2.cmd.run(
            lambda: self.vision_system.periodic(), [self.vision_system]))

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OIConstants.kOperatorControllerPort, "xbox")
        self.driver_controller = self.driver_controller_raw.get_controller()  # TODO delete this please
        DriverStation.silenceJoystickConnectionWarning(True)

        # Run routine to connect controller buttons to command input into the scheduler.
        self.configureButtonBindings()

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

        # Set default arm command.
        self.arm.setDefaultCommand(commands2.cmd.run(lambda: self.arm.set_setpoint("stow"), [self.arm]))

        # Setup for all event-trigger commands.
        self.configureTriggers()

        # Setup autonomous selector on the dashboard.
        self.m_chooser = SendableChooser()
        self.m_chooser.setDefaultOption("No-op", "No-op")
        self.m_chooser.addOption("Score_Collect_Score_Balance", "Score_Collect_Score_Balance")
        self.m_chooser.addOption("Simple Path", "Simple Path")
        self.m_chooser.addOption("Test Commmands", "Test Commands")
        self.m_chooser.addOption("QuikAuto", "QuikAuto")
        self.m_chooser.addOption("s_c_s_c_b_s", "s_c_s_c_b_s")
        SmartDashboard.putData("Auto Select", self.m_chooser)

        # Create a boolean on the dashboard to reset pose without enabling.
        SmartDashboard.putData("Reset Pose", commands2.cmd.runOnce(lambda: self.vision_system.reset_hard_odo(),
                                                                   [self.vision_system, self.robot_drive]))

        # Create a command on the dashboard to enable "Debug Mode" for the drivetrain.
        # SmartDashboard.putData("Debug Mode On", commands2.cmd.runOnce(lambda: self.robot_drive.debug_toggle(True),
        #                                                               [self.robot_drive]))
        SmartDashboard.putData("Debug Mode On", DebugMode(self.robot_drive, True))
        # SmartDashboard.putData("Debug Mode Off", commands2.cmd.runOnce(lambda: self.robot_drive.debug_toggle(False),
        #                                                                [self.robot_drive]))
        SmartDashboard.putData("Debug Mode Off", DebugMode(self.robot_drive, False))

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # Press "A" on pilot controller to reset IMU heading
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("A")).whenPressed(
            commands2.cmd.runOnce(lambda: self.robot_drive.zero_heading(), [self.robot_drive]))

        # Press "X" on pilot controller to clear the currently running LED pattern
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("X")).whenPressed(
            commands2.cmd.runOnce(lambda: self.leds.clear_pattern, [self.leds]))

        # Hold left trigger on pilot controller to enable drive lock
        commands2.button.Button(lambda: self.driver_controller_raw.get_trigger("L", 0.5)).whenHeld(
            commands2.cmd.run(lambda: self.robot_drive.drive_lock(), [self.robot_drive]))

        # Hold right trigger on pilot controller to enable slow mode at 50%
        commands2.button.Button(lambda: self.driver_controller_raw.get_trigger("R", 0.1)).whenHeld(
            commands2.cmd.run(lambda: self.robot_drive.drive_slow(
                self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                self.driver_controller.getRightX() * DriveConstants.kMaxAngularSpeed,
                True,
                0.5), [self.robot_drive]))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        commands2.button.Button(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                -90,
            ), [self.robot_drive]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                0
            ), [self.robot_drive]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                90
            ), [self.robot_drive]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                180
            ), [self.robot_drive]))

        # Reset robot perceived pose based on current vision data
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("Y")).whenHeld(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(), [self.vision_system])
        )

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "Simple Path":
            return autoplays.simple_path(self.robot_drive, self.leds)
        elif self.m_chooser.getSelected() == "Test Commands":
            return autoplays.test_commands(self.robot_drive, self.leds)
        elif self.m_chooser.getSelected() == "No-op":
            return None
        elif self.m_chooser.getSelected() == "QuikAuto":
            return autoplays.quik_auto(self.robot_drive, self.leds)
        elif self.m_chooser.getSelected() == "s_c_s_c_b_s":
            return autoplays.s_c_s_c_b_s(self.robot_drive, self.leds, self.arm, self.intake)
        else:
            return None

    def configureTriggers(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        commands2.Trigger(lambda: self.vision_system.has_targets()).toggleOnFalse(
            NotifierLEDs(self.leds, "RED", self.leds.current_state))
        commands2.Trigger(lambda: self.vision_system.has_targets()).toggleOnTrue(
            NotifierLEDs(self.leds, "GREEN", self.leds.current_state))
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("B")).toggleOnTrue(
            commands2.cmd.run(lambda: self.vision_system.toggle_leds(True), [self.vision_system]))
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("B")).toggleOnFalse(
            commands2.cmd.run(lambda: self.vision_system.toggle_leds(False), [self.vision_system]))

        # When VIEW is pressed on driver controller, enable auto balancing.
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("VIEW")).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.auto_balance(-1), [self.robot_drive]))

        # When RT is pressed, put the arm and intake to their setpoints for intaking.
        commands2.Trigger(lambda: self.operator_controller_raw.get_trigger("R", 0.3)).whileTrue(
            commands2.ParallelCommandGroup(
                commands2.cmd.run(lambda: self.arm.set_setpoint("intake"), [self.arm]),
                commands2.cmd.run(lambda: self.intake.intake(False, 1), [self.intake])
            )
        )
        # When LT is pressed, the intake spits out anything it has in it without moving the arm
        commands2.Trigger(lambda: self.operator_controller_raw.get_trigger("L", 0.3)).whileTrue(
            commands2.cmd.run(lambda: self.intake.intake(True, 1), [self.intake]))

        # When up on the D-pad is pressed, put the arm into high back position
        commands2.Trigger(lambda: self.operator_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.arm.set_setpoint("shoot_high_back"), [self.arm]))
        # commands2.Trigger(lambda: wpilib.RobotState.isTeleop()).toggleOnTrue(
        #     commands2.cmd.run(lambda: self.utilsys.toggle_channel(True), [self.utilsys]))
        # commands2.Trigger(lambda: wpilib.RobotState.isDisabled()).toggleOnTrue(
        #     commands2.cmd.run(lambda: self.utilsys.toggle_channel(True), [self.utilsys]))

        # TODO Fix the snapping to zero direction
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("MENU")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.dir_est_ctrl("R")
            ))
        )
