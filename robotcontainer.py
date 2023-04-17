import commands2
import commands2.button
import commands2.cmd
import wpilib

from constants import OIConstants, DriveConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from wpilib import SmartDashboard, SendableChooser
import autoplays
from commands.default_leds import DefaultLEDs
from helpers.custom_hid import CustomHID


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robot_drive = DriveSubsystem()
        self.leds = LEDs(0, 145, 0.05)
        # The following line is intended to implement vision based pose estimation
        # self.vision_system = VisionSubsystem()

        # The driver's controller
        self.driver_controller_raw = CustomHID(0, "xbox")
        self.driver_controller = self.driver_controller_raw.get_xbox_controller()

        # Configure the button bindings
        self.configureButtonBindings()

        self.robot_drive.setDefaultCommand(commands2.cmd.run(
            lambda: self.robot_drive.drive(self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                                           self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                                           self.driver_controller.getRightX() * DriveConstants.kMaxAngularSpeed,
                                           True,
                                           False),
            [self.robot_drive]
        ))

        # Here's some code to adapt the current drive command to the new CustomHID layout. Need real robot to test,
        # so commented out for now.
        # self.robot_drive.setDefaultCommand(commands2.cmd.run(
        #     lambda: self.robot_drive.drive(self.driver_controller_raw.get_axis("LY", 0.07) * DriveConstants.kMaxSpeed,
        #                                    self.driver_controller_raw.get_axis("LX", 0.07) * DriveConstants.kMaxSpeed,
        #                                    self.driver_controlle_raw.get_axis("RX", 0.07) * DriveConstants.kMaxAngularSpeed,
        #                                    True,
        #                                    False),
        #     [self.robot_drive]
        # ))

        self.leds.setDefaultCommand(DefaultLEDs(self.leds))

        # SmartDashboard.putData(commands2.CommandScheduler.getInstance())
        self.m_chooser = SendableChooser()
        self.m_chooser.setDefaultOption("No-op", "No-op")
        self.m_chooser.addOption("Leave_Community", "Leave_Community")
        SmartDashboard.putData("Auto Select", self.m_chooser)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        commands2.button.Button(lambda: self.driver_controller_raw.get_button("A")).whenPressed(
            commands2.cmd.runOnce(lambda: self.robot_drive.zero_heading(), [self.robot_drive]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("B")).whenPressed(
            commands2.cmd.run(lambda: self.leds.rainbow_shift(True), [self.leds]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("X")).whenPressed(
            commands2.cmd.runOnce(lambda: self.leds.rainbow_shift(False), [self.leds]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_button("Y")).whenPressed(
            commands2.cmd.run(lambda: self.leds.heading_lock(self.robot_drive.get_heading()), [self.leds]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_trigger("L", 0.5)).whenHeld(
            commands2.cmd.run(lambda: self.robot_drive.drive_lock(), [self.robot_drive]))
        commands2.button.Button(lambda: self.driver_controller_raw.get_trigger("R", 0.1)).whenHeld(
            commands2.cmd.run(lambda: self.robot_drive.drive_slow(
                self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                self.driver_controller.getRightX() * DriveConstants.kMaxAngularSpeed,
                True,
                True,
                0.5), [self.robot_drive]))

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main {@link Robot} class.
        :returns: the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "Leave_Community":
            return autoplays.path_planner_test(self.robot_drive)
        else:
            return None
