// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d rollPosition = new Rotation2d();
        public Rotation2d pitchPosition = new Rotation2d();
        public Rotation2d yawPosition = new Rotation2d();
        public double rollVelocity = 0.0;
        public double pitchVelocity = 0.0;
        public double yawVelocity = 0.0;
        public boolean isCalibrating = true;
    }

    /* Zeroes the gyro. */
    public default void zeroGyro() {}

    public default boolean isCalibrating() {
        return false;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}