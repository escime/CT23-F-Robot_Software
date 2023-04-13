"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
# import wpimath.units


class DriveConstants:
    kEncoderResolution = 4096
    kWheelDiameterInches = 4
    kWheelRadius = 4*0.0254*0.5
    kModuleMaxAngularVelocity = 3  # ????
    kModuleMaxAngularAcceleration = 2 * math.pi
    t_velocity_conversion_factor = 0.00488
    # 1/21.42857142857143
    t_position_conversion_factor = 0.2932  # L2 ratio is 21.42857142857143
    d_velocity_conversion_factor = 0.0007885761
    # (1/6.746031745) * 0.319185544
    d_position_conversion_factor = 0.047314566  # L2 ratio is 6.746031745
    kMaxSpeed = 10  # Set max speed in m/s
    kMaxAngularSpeed = 11  # Set max rotation speed rot/s
    kGyroReversed = False

    m_FL_location = Translation2d(0.289, 0.289)  # position of wheel center in meters
    m_FR_location = Translation2d(0.289, -0.289)
    m_BL_location = Translation2d(-0.289, 0.289)
    m_BR_location = Translation2d(-0.289, -0.289)
    m_kinematics = SwerveDrive4Kinematics(m_FL_location, m_FR_location, m_BL_location, m_BR_location)


class AutoConstants:
    kAutoTimeoutSeconds = 12
    kAutoShootTimeSeconds = 7
    kMaxSpeedMetersPerSecond = 4.5
    kMaxAccelerationMetersPerSecondSquared = 10.0

    kPXController = 1
    kPYController = 1
    kPThetaController = 1
    kThetaControllerConstraints = TrapezoidProfile.Constraints(2 * math.pi, 2 * math.pi)


class OIConstants:
    kDriverControllerPort = 0


class ModuleConstants:
    oops = 14
