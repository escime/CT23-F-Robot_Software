import commands2
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from subsystems.drivesubsystem import DriveSubsystem
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
import math


class VisionSubsystem(commands2.SubsystemBase):
    limelight_table: NetworkTableInstance.getDefault().getTable("limelight")
    tv = 0.0
    ta = 0.0
    tl = 0.0
    json_val = {}
    timestamp = 0
    target_led_mode = 1
    pip_mode = 2
    target_tag = 2

    def __init__(self, robot_drive: DriveSubsystem) -> None:
        super().__init__()
        self.robot_drive = robot_drive  # This is structurally not great but necessary for certain features.
        self.limelight_table = NetworkTableInstance.getDefault().getTable("limelight")

    def toggle_leds(self, on: bool):
        if on:
            self.limelight_table.putNumber("ledMode", 3.0)
            return True
        else:
            self.limelight_table.putNumber("ledMode", 1.0)
            return False

    def update_values(self):
        """Update relevant values from LL NT to robot variables."""
        self.tv = self.limelight_table.getEntry("tv").getDouble(0)  # Get "target acquired" boolean as a 1.0 or 0.0.
        self.ta = self.limelight_table.getEntry("ta").getDouble(0)  # Get "target area of image" as a double.
        self.tl = self.limelight_table.getEntry("tl").getDouble(0)  # Get pipeline latency contribution. Unused.
        self.json_val = self.limelight_table.getEntry("json").getString("0")  # Grab the entire json pull as a string.
        first_index = str(self.json_val).find("\"ts\"")  # Locate the first string index for timestamp.
        adjusted_json = str(self.json_val)[first_index + 5:]  # Substring the JSON to remove everything before timestamp
        timestamp_str = adjusted_json[:adjusted_json.find(",")]  # Substring out the timestamp.
        try:
            self.timestamp = float(timestamp_str)  # Update timestamp if JSON parse is successful.
        except ValueError:
            self.timestamp = -1

    def has_targets(self) -> bool:
        """Checks if the limelight can see a target."""
        if self.tv == 1:
            return True
        else:
            return False

    def vision_estimate_pose(self):
        """Acquires limelight estimated robot pose. Currently, pulls wpiblue botpose only."""
        botpose = self.limelight_table.getEntry("botpose_wpiblue").getDoubleArray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        bot_x = botpose[0]
        bot_y = botpose[1]
        rotation_z = (botpose[5] + 360) % 360

        return Pose2d(Translation2d(bot_x, bot_y), Rotation2d.fromDegrees(rotation_z))

#     def get_latency(self):
#         return Timer.getFPGATimestamp() - wpimath.units.millisecondsToSeconds(self.tl)

    def reset_hard_odo(self):
        """Reset robot odometry based on vision pose. Intended for use only during testing, since there is no auto
        to automatically update the initial pose and the software assumes (0, 0)."""
        # self.robot_drive.reset_odometry(self.vision_estimate_pose())
        self.robot_drive.reset_odometry(Pose2d(Translation2d(8.12, 4), Rotation2d(0)))

    def periodic(self) -> None:
        """Update vision variables and robot odometry as fast as scheduler allows."""
        # self.update_values()

        if self.limelight_table.getNumber("stream", -1) != self.pip_mode:
            self.limelight_table.putNumber("stream", self.pip_mode)

        # if self.has_targets():
            # current_position = self.robot_drive.get_pose()
            # vision_estimate = self.vision_estimate_pose()
            # SmartDashboard.putString("Vision Estimated Pose", str(vision_estimate))

            # if abs(current_position.x - vision_estimate.x) < 1 and \
            #         abs(current_position.y - vision_estimate.y) < 1:  # Sanity check for pose updates.
            #     self.robot_drive.add_vision(vision_estimate, self.timestamp)

    def update_target_tag(self, target: int) -> None:
        """Set the VisionSubsystem's target apriltag based on the red alliance equivalent tags."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if target == 1:
                self.target_tag = 6
            elif target == 2:
                self.target_tag = 7
            elif target == 3:
                self.target_tag = 8
            elif target == 5:
                self.target_tag = 4
        else:
            self.target_tag = target

    def generate_path_to_tag(self) -> Trajectory:
        estimate_config = TrajectoryConfig(4, 3)
        end_pose = self.robot_drive.get_pose()
        if self.target_tag == 1:
            end_pose = Pose2d(14.67, 0.94, math.pi)
        elif self.target_tag == 2:
            end_pose = Pose2d(14.67, 2.62, math.pi)
        elif self.target_tag == 3:
            end_pose = Pose2d(14.67, 4.24, math.pi)
        elif self.target_tag == 4:
            end_pose = Pose2d(15.64, 6.68, math.pi)
        elif self.target_tag == 5:
            end_pose = Pose2d(0.69, 6.68, 180)
        elif self.target_tag == 6:
            end_pose = Pose2d(1.68, 4.24, 180)
        elif self.target_tag == 7:
            end_pose = Pose2d(1.68, 2.62, 180)
        elif self.target_tag == 8:
            end_pose = Pose2d(1.68, 0.94, 180)
        path = TrajectoryGenerator.generateTrajectory([self.robot_drive.get_pose(), end_pose], estimate_config)
        return path
