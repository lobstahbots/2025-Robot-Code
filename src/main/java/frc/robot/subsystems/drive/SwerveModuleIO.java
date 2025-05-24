// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public Rotation2d drivePosition = new Rotation2d();
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public Rotation2d angularOffset = new Rotation2d();
    }

    public default void updateInputs(ModuleIOInputs inputs) {};

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(double volts) {};

    /** Run the turn motor at the specified voltage. */
    public default void setTurnVoltage(double volts) {};

    /** Set the angle to the angle specified in the module state. */
    public default void setAngle(SwerveModuleState optimizedDesiredState) {}

    /** Set the drive speed to the angle specified in the module state. */
    public default void setDriveSpeed(SwerveModuleState optimizedDesiredState, boolean isOpenLoop) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveIdleMode(IdleMode mode) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setTurnIdleMode(IdleMode mode) {}

    public default void periodic() {}

}
