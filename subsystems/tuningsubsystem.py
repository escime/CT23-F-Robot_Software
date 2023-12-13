import commands2
from rev import CANSparkMax
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from constants import ModuleConstants, DriveConstants


class TuningSubsystem(commands2.Subsystem):
    fl_drive = CANSparkMax(ModuleConstants.fl_drive_id, CANSparkMax.MotorType.kBrushless)
    fr_drive = CANSparkMax(ModuleConstants.fr_drive_id, CANSparkMax.MotorType.kBrushless)
    bl_drive = CANSparkMax(ModuleConstants.bl_drive_id, CANSparkMax.MotorType.kBrushless)
    br_drive = CANSparkMax(ModuleConstants.br_drive_id, CANSparkMax.MotorType.kBrushless)

    fl_turn = CANSparkMax(ModuleConstants.fl_turn_id, CANSparkMax.MotorType.kBrushless)
    fr_turn = CANSparkMax(ModuleConstants.fr_turn_id, CANSparkMax.MotorType.kBrushless)
    bl_turn = CANSparkMax(ModuleConstants.bl_turn_id, CANSparkMax.MotorType.kBrushless)
    br_turn = CANSparkMax(ModuleConstants.br_turn_id, CANSparkMax.MotorType.kBrushless)

    fl_drive_enc = fl_drive.getEncoder()
    fr_drive_enc = fr_drive.getEncoder()
    bl_drive_enc = bl_drive.getEncoder()
    br_drive_enc = br_drive.getEncoder()

    def __init__(self, drive: bool, single_motor: bool, canid: int) -> None:
        super().__init__()
        if single_motor:
            self.single_motor = CANSparkMax(canid, CANSparkMax.MotorType.kBrushless)
            self.single_motor_enc = self.single_motor.getEncoder()

        if drive:
            self.fl_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.fr_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.bl_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.br_drive_enc.setVelocityConversionFactor(DriveConstants.d_velocity_conversion_factor)
            self.fl_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)
            self.fr_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)
            self.bl_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)
            self.br_drive_enc.setPositionConversionFactor(DriveConstants.d_position_conversion_factor)

            self.fl_drive.setInverted(True)
            self.fr_drive.setInverted(True)
            self.bl_drive.setInverted(True)
            self.br_drive.setInverted(True)
            self.fl_turn.setInverted(True)
            self.fr_turn.setInverted(True)
            self.bl_turn.setInverted(True)
            self.br_turn.setInverted(True)

            self.fl_turn_pid = PIDController(DriveConstants.azimuth_controller_PID[0],
                                             DriveConstants.azimuth_controller_PID[1],
                                             DriveConstants.azimuth_controller_PID[2])
            self.fr_turn_pid = PIDController(DriveConstants.azimuth_controller_PID[0],
                                             DriveConstants.azimuth_controller_PID[1],
                                             DriveConstants.azimuth_controller_PID[2])
            self.bl_turn_pid = PIDController(DriveConstants.azimuth_controller_PID[0],
                                             DriveConstants.azimuth_controller_PID[1],
                                             DriveConstants.azimuth_controller_PID[2])
            self.br_turn_pid = PIDController(DriveConstants.azimuth_controller_PID[0],
                                             DriveConstants.azimuth_controller_PID[1],
                                             DriveConstants.azimuth_controller_PID[2])

            self.fl_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.fr_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.bl_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.br_drive.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.fl_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.fr_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.bl_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)
            self.br_turn.setIdleMode(CANSparkMax.IdleMode.kBrake)

            self.fl_drive.burnFlash()
            self.fr_drive.burnFlash()
            self.bl_drive.burnFlash()
            self.br_drive.burnFlash()
            self.fl_turn.burnFlash()
            self.fr_turn.burnFlash()
            self.bl_turn.burnFlash()
            self.br_turn.burnFlash()

    def id_ks_sm(self, threshold: float) -> None:
        """Identify the kS term for a single motor."""
        ks_found = False
        voltage = 0
        ks = 0
        self.single_motor_enc.setPosition(0)
        while not ks_found:
            self.single_motor.setVoltage(voltage)

            if self.single_motor_enc.getPosition() > threshold:
                ks_found = True
                ks = voltage
            else:
                voltage += 0.01
        self.single_motor.setVoltage(0)
        SmartDashboard.putNumber("Calculated kS", ks)
        print(ks)

    def id_ks_dt(self, threshold: float) -> None:
        """Identify the kS term for the drivetrain."""
        ks_found = [False, False, False, False]
        voltages = [0, 0, 0, 0]
        ks = [0, 0, 0, 0]
        self.fl_drive_enc.setPosition(0)
        self.fr_drive_enc.setPosition(0)
        self.bl_drive_enc.setPosition(0)
        self.br_drive_enc.setPosition(0)
        while False in ks_found:
            if not ks_found[0]:
                self.fl_drive.setVoltage(voltages[0])
            if not ks_found[1]:
                self.fr_drive.setVoltage(voltages[1])
            if not ks_found[2]:
                self.bl_drive.setVoltage(voltages[2])
            if not ks_found[3]:
                self.br_drive.setVoltage(voltages[3])

            if self.fl_drive_enc.getPosition() > threshold:
                ks_found[0] = True
                ks[0] = True
                self.fl_drive.setVoltage(0)
            else:
                voltages[0] += 0.01
            if self.fr_drive_enc.getPosition() > threshold:
                ks_found[1] = True
                ks[1] = True
                self.fr_drive.setVoltage(0)
            else:
                voltages[1] += 0.01
            if self.bl_drive_enc.getPosition() > threshold:
                ks_found[2] = True
                ks[2] = True
                self.bl_drive.setVoltage(0)
            else:
                voltages[2] += 0.01
            if self.br_drive_enc.getPosition() > threshold:
                ks_found[3] = True
                ks[3] = True
                self.br_drive.setVoltage(0)
            else:
                voltages[3] += 0.01
        SmartDashboard.putNumber("Calculated kS", max(ks))
        print(max(ks))

    def id_kv_sm(self, voltage):
        stabilized = False
        last_vel = 0
        while not stabilized:
            self.single_motor.setVoltage(voltage)
            if self.single_motor_enc.getVelocity() <= last_vel:
                stabilized = True
            last_vel = self.single_motor_enc.getVelocity()
        kv = last_vel / voltage
        self.single_motor.setVoltage(0)
        SmartDashboard.putNumber("Calculated kV", kv)
        print(kv)

    def id_kv_dt(self, voltage: float) -> None:
        """Identify the kS term for the drivetrain."""
        kv = [0, 0, 0, 0]
        last_vel = [0, 0, 0, 0]
        stabilized = [False, False, False, False]
        while False in stabilized:
            self.fl_drive.setVoltage(voltage)
            self.fr_drive.setVoltage(voltage)
            self.bl_drive.setVoltage(voltage)
            self.br_drive.setVoltage(voltage)

            if self.fl_drive_enc.getVelocity() <= last_vel[0]:
                stabilized[0] = True
            if self.fr_drive_enc.getVelocity() <= last_vel[1]:
                stabilized[1] = True
            if self.bl_drive_enc.getVelocity() <= last_vel[2]:
                stabilized[2] = True
            if self.fl_drive_enc.getVelocity() <= last_vel[3]:
                stabilized[3] = True

            last_vel[0] = self.fl_drive_enc.getVelocity()
            last_vel[1] = self.fr_drive_enc.getVelocity()
            last_vel[2] = self.bl_drive_enc.getVelocity()
            last_vel[3] = self.br_drive_enc.getVelocity()

        self.fl_drive.setVoltage(0)
        self.fr_drive.setVoltage(0)
        self.bl_drive.setVoltage(0)
        self.br_drive.setVoltage(0)

        kv[0] = last_vel[0] / voltage
        kv[1] = last_vel[1] / voltage
        kv[2] = last_vel[2] / voltage
        kv[3] = last_vel[3] / voltage
        kv_avg = sum(kv) / len(kv)

        SmartDashboard.putNumber("Calculated kV", kv_avg)
        print(kv_avg)
