// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroIONavX implements GyroIO {
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    public GyroIONavX(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
      }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(-gyro.getPitch());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(-gyro.getRoll());
    }

    public double getYawVelocity() {
        return gyro.getVelocityZ();
    }

    public double getRollVelocity() {
        return gyro.getVelocityY();
    }

    public double getPitchVelocity() {
        return gyro.getVelocityX();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.rollPosition = getRoll();
        inputs.pitchPosition = getPitch();
        inputs.yawPosition = getYaw();
        inputs.rollVelocity = getRollVelocity();
        inputs.pitchVelocity = getPitchVelocity();
        inputs.yawVelocity = getYawVelocity();
    }
}
