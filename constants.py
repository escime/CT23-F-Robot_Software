"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
import wpimath.units


class DriveConstants:
    wheel_diameter = wpimath.units.inchesToMeters(4)
    wheel_circumference = wheel_diameter * math.pi
    drive_gear_ratio = 6.75
    angle_gear_ratio = 21.43

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
    kMaxSpeed = 10  # Set max speed in m/s 10
    kMaxAngularSpeed = 20  # Set max rotation speed rot/s 20
    kGyroReversed = False

    m_FL_location = Translation2d(0.244, 0.244)
    m_FR_location = Translation2d(0.244, -0.244)
    m_BL_location = Translation2d(-0.244, 0.244)
    m_BR_location = Translation2d(-0.244, -0.244)
    m_kinematics = SwerveDrive4Kinematics(m_FL_location, m_FR_location, m_BL_location, m_BR_location)

    snap_controller_PID = [0.17, 0, 0]
    drive_controller_PID = [1.5, 0, 0]
    azimuth_controller_PID = [2, 0, 0]
    drive_controller_FF = [0.22/12, 1.0/12, 0.23/12]

    # Here are the sysID constants :) They felt less snappy to me so they just live here now
    # drive_controller_PID = [0.32069, 0, 0]
    # drive_controller_FF = [0.10338, 2.6387, 0.17711]

    closed_loop_ramp = 0.0
    open_loop_ramp = 0.25
    drive_current_limit = 38
    azimuth_current_limit = 38

    balance_PID = [0.01, 0, 0]

    slew_rate_drive = 50
    slew_rate_turn = 0


class AutoConstants:
    kMaxSpeedMetersPerSecond = 4.0
    kMaxAccelerationMetersPerSecondSquared = 3.0

    kPXController = 10
    kPYController = 10
    kPThetaController = 10
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(kMaxSpeedMetersPerSecond,
                                                                      kMaxAccelerationMetersPerSecondSquared)
    max_module_speed = 4.5
    module_radius_from_center = 0.3451


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1


class ModuleConstants:
    fl_drive_id = 10
    fl_turn_id = 11
    fl_encoder_id = 12
    fl_zero_offset = -264.11

    fr_drive_id = 13
    fr_turn_id = 14
    fr_encoder_id = 15
    fr_zero_offset = -297.77

    br_drive_id = 16
    br_turn_id = 17
    br_encoder_id = 18
    br_zero_offset = -77.34

    bl_drive_id = 19
    bl_turn_id = 20
    bl_encoder_id = 21
    bl_zero_offset = -160.49


class ArmConstants:
    kP = 0.2
    kI = 0
    kD = 0
    kFF = 0
    kMinOutput = -0.3
    kMaxOutput = 0.3
    maxVel = 0
    maxAcc = 0
    minVel = 0
    allowedErr = 0

    masterControlID = 30
    velocityConversion = 1
    positionConversion = 1


class IntakeConstants:
    current_limit = 30
    motor_id = 31
    high_front_power = 1
    mid_front_power = 1
    high_back_power = 1
    mid_back_power = 1
    armed_speed = 0
