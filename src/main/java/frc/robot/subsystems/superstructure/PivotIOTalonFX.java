// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PivotConstants;

public class PivotIOTalonFX implements PivotIO {

    private final TalonFX pivotMotor;
    private final Canandmag encoder;

    /** Creates a new Pivot. */
    public PivotIOTalonFX(int pivotMotorID, int encoderID) {
        this.encoder = new Canandmag(encoderID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        pivotMotor = new TalonFX(pivotMotorID);
        config.CurrentLimits.SupplyCurrentLimit = PivotConstants.CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = PivotConstants.PIVOT_GEARING;
        // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 
        // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.MAX_ANGLE;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.MIN_ANGLE;
        pivotMotor.getConfigurator().apply(config);
        pivotMotor.setPosition(encoder.getAbsPosition());
    }

    public void setIdleMode(NeutralModeValue idleMode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = idleMode;
        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        pivotMotor.stopMotor();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.position = Rotation2d.fromRotations(encoder.getAbsPosition());
        inputs.velocity = encoder.getVelocity();
        inputs.supplyCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.torqueCurrent = pivotMotor.getTorqueCurrent().getValueAsDouble();
        inputs.temperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.appliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    }
}
