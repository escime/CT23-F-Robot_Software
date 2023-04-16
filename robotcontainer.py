import commands2
import commands2.button
import commands2.cmd
import wpilib

from constants import OIConstants, DriveConstants
# from commands.defaultdrive import DefaultDrive
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.leds import LEDs
from wpilib import SmartDashboard, SendableChooser
import autoplays
from commands.default_leds import DefaultLEDs


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
        self.leds = LEDs(0, 150, 0.05)
        # The following line is intended to implement vision based pose estimation
        # self.vision_system = VisionSubsystem()

        # The driver's controller
        self.driver_controller = wpilib.XboxController(OIConstants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Set the default drive command
        # self.robot_drive.setDefaultCommand(
        #         DefaultDrive(self.robot_drive,
        #                      self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
        #                      self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
        #                      self.driver_controller.getRightX() * DriveConstants.kMaxAngularSpeed,
        #                      False)
        # )

        self.robot_drive.setDefaultCommand(commands2.cmd.run(
            lambda: self.robot_drive.drive(self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed,
                                           self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed,
                                           self.driver_controller.getRightX() * DriveConstants.kMaxAngularSpeed,
                                           True,
                                           True),
            [self.robot_drive]
        ))

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
        # All of this code is sort of hack-and-slashed together to make it happen the easy way. All of these commands
        # should be refactored as their own files with their own command parameters - it's not good form to create them
        # all in-line as I've done here. It just takes more time and this was a good way to test the commmands
        # themselves were actually working. If I keep any of these, I'll probably fix this section.
        commands2.button.JoystickButton(self.driver_controller, 1).whenPressed(
            commands2.cmd.runOnce(lambda: self.robot_drive.zero_heading(), [self.robot_drive]))
        commands2.button.JoystickButton(self.driver_controller, 2).whenPressed(
            commands2.cmd.run(lambda: self.leds.rainbow_shift(True), [self.leds]))
        commands2.button.JoystickButton(self.driver_controller, 3).whenPressed(
            commands2.cmd.runOnce(lambda: self.leds.rainbow_shift(False), [self.leds]))
        commands2.button.JoystickButton(self.driver_controller, 4).whenPressed(
            commands2.cmd.run(lambda: self.leds.heading_lock(self.robot_drive.get_heading()), [self.leds]))
        commands2.button.JoystickButton(self.driver_controller, 6).whenHeld(
            commands2.cmd.run(lambda: self.robot_drive.drive_lock(), [self.robot_drive]))
        commands2.button.JoystickButton(self.driver_controller, 5).whenHeld(commands2.cmd.run(
            lambda: self.robot_drive.drive(self.driver_controller.getLeftY() * DriveConstants.kMaxSpeed / 2,
                                           self.driver_controller.getLeftX() * DriveConstants.kMaxSpeed / 2,
                                           self.driver_controller.getRightX() * DriveConstants.kMaxAngularSpeed / 2,
                                           True,
                                           True),
            [self.robot_drive]
        ))

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main {@link Robot} class.
        :returns: the command to run in autonomous
        """
        if self.m_chooser.getSelected() == "Leave_Community":
            return autoplays.path_planner_test(self.robot_drive)
        else:
            return None
