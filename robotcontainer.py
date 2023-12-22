import commands2
import commands2.button
import commands2.cmd

from constants import OIConstants
from subsystems.leds import LEDs
from wpilib import SmartDashboard, SendableChooser
from commands.default_leds import DefaultLEDs
from commands.shoot_leds import ShootLEDs
from helpers.custom_hid import CustomHID


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.leds = LEDs(0, 45, 1, 0.03, "GRB")

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OIConstants.kDriverControllerPort, "xbox")

        # Set default LED command.
        self.leds.setDefaultCommand(DefaultLEDs(self.leds))

        # Setup for all event-trigger commands.
        self.configureTriggers()

        # Setup autonomous selector on the dashboard.
        self.m_chooser = SendableChooser()
        self.m_chooser.setDefaultOption("No-op", "No-op")
        SmartDashboard.putData("Auto Select", self.m_chooser)

    def getAutonomousCommand(self) -> commands2.cmd:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        return None

    def configureTriggers(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("A")).toggleOnTrue(
            ShootLEDs(self.leds, "default"))
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("B")).toggleOnTrue(
            ShootLEDs(self.leds, "fast"))
        commands2.Trigger(lambda: self.driver_controller_raw.get_button("X")).toggleOnTrue(
            ShootLEDs(self.leds, "fastest"))
