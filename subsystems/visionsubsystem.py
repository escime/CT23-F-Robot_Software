import commands2
import wpimath.units
from ntcore import NetworkTableInstance
from wpilib import DriverStation, Timer, SmartDashboard, Field2d
# from wpilib.shuffleboard import Shuffleboard
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Transform2d
from subsystems.drivesubsystem import DriveSubsystem
import math


class VisionSubsystem(commands2.SubsystemBase):
    limelight_table: NetworkTableInstance.getDefault().getTable("limelight")
    tv = 0.0
    ta = 0.0
    tl = 0.0
    m_field = Field2d()

    def __init__(self, robot_drive: DriveSubsystem) -> None:
        super().__init__()
        self.robot_drive = robot_drive
        self.limelight_table = NetworkTableInstance.getDefault().getTable("limelight")

    def toggle_leds(self, on):
        if on:
            self.limelight_table.putNumber("ledMode", 3)
            return True
        else:
            self.limelight_table.putNumber("ledMode", 1)
            return False

    def update_values(self):
        self.tv = self.limelight_table.getEntry("tv").getDouble(0)
        self.ta = self.limelight_table.getEntry("ta").getDouble(0)
        self.tl = self.limelight_table.getEntry("tl").getDouble(0)

    def has_targets(self):
        if self.tv == 1:
            return True
        else:
            return False

    def vision_estimate_pose(self):
        botpose = self.limelight_table.getEntry("botpose_wpiblue").getDoubleArray([0.0, 0.0])

        bot_x = botpose[0]
        bot_y = botpose[1]
        rotation_z = (botpose[5] + 360) % 360

        return Pose2d(Translation2d(bot_x, bot_y), Rotation2d.fromDegrees(rotation_z))

#     def get_latency(self):
#         return Timer.getFPGATimestamp() - wpimath.units.millisecondsToSeconds(self.tl)

    def periodic(self) -> None:
        self.update_values()

        if self.has_targets():
            current_position = self.robot_drive.get_pose()
            vision_estimate = self.vision_estimate_pose()
            SmartDashboard.putString("Vision Estimated Pose", str(vision_estimate))
            self.m_field.setRobotPose(vision_estimate)
            SmartDashboard.putData("Field", self.m_field)
    #
    #         if abs(current_position.x() - vision_estimate.x()) < 1 and \
    #                 abs(current_position.y() - vision_estimate.y()) < 1:
    #             DriveSubsystem.reset_odometry(self, vision_estimate)
