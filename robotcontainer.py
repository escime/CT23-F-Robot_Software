import commands2
import commands2.button
import commands2.cmd
from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from subsystems.visionsubsystem import VisionSubsystem
from wpilib import SmartDashboard, SendableChooser
import autoplays
from commands.default_leds import DefaultLEDs
from commands.notifier_led import NotifierLEDs
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

        self.vision_system.setDefaultCommand(commands2.cmd.run(
            lambda: self.vision_system.periodic(), [self.vision_system]))
        commands2.cmd.runOnce(lambda: self.vision_system.toggle_leds(True), [self.vision_system])

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")
        self.driver_controller = self.driver_controller_raw.get_controller()  # Retained for legacy support.

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

        # Setup for connecting the vision system to the LED notifier system.
        commands2.Trigger(lambda: self.vision_system.has_targets()).onTrue(
            NotifierLEDs(self.leds, "GREEN", self.leds.current_state))
        commands2.Trigger(lambda: self.vision_system.has_targets()).onFalse(
            NotifierLEDs(self.leds, "RED", self.leds.current_state))

        # Setup autonomous selector on the dashboard.
        self.m_chooser = SendableChooser()
        self.m_chooser.setDefaultOption("No-op", "No-op")
        self.m_chooser.addOption("Score_Collect_Score_Balance", "Score_Collect_Score_Balance")
        self.m_chooser.addOption("Simple Path", "Simple Path")
        SmartDashboard.putData("Auto Select", self.m_chooser)

        # Push an update to the limelight NT to turn off the LEDs.
        # TODO Check if this is actually working. Last I checked, I wasn't able to push anything to NT via this.
        commands2.cmd.runOnce(lambda: self.vision_system.toggle_leds(False), [self.vision_system]).schedule()

        # Create a boolean on the dashboard to reset pose without enabling. Will need separated command.
        SmartDashboard.setDefaultBoolean("Reset Pose", False)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # Press "A" on pilot controller to reset IMU heading
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("A")).whenPressed(
            commands2.cmd.runOnce(lambda: self.robot_drive.zero_heading(), [self.robot_drive]))

        # Press "B" on pilot controller to set LEDs into "Rainbow Shift" pattern
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("B")).whenPressed(
            commands2.cmd.run(lambda: self.leds.rainbow_shift(), [self.leds]))

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
                90
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
                -90
            ), [self.robot_drive]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                180
            ), [self.robot_drive]))

        # Reset robot perceived pose based on current vision data
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("Y")).whenPressed(
            commands2.cmd.run(lambda: self.vision_system.reset_hard_odo(), [self.vision_system])
        )

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "Score_Collect_Score_Balance":
            return autoplays.path_points_test(self.robot_drive, self.leds)
        elif self.m_chooser.getSelected() == "Simple Path":
            return autoplays.simple_path(self.robot_drive, self.leds)
        else:
            return None
