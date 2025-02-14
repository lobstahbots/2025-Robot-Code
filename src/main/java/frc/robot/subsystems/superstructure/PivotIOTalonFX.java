// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PivotConstants;

public class PivotIOTalonFX implements PivotIO {

    private final TalonFX pivotMotor;
    private final AbsoluteEncoder encoder;

    /** Creates a new Pivot. */
    public PivotIOTalonFX(int pivotMotorID, AbsoluteEncoder encoder) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        pivotMotor = new TalonFX(pivotMotorID);
        config.CurrentLimits.SupplyCurrentLimit = PivotConstants.CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().apply(config);
        this.encoder = encoder;
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
        inputs.position = new Rotation2d(encoder.getPosition());
        inputs.velocity = pivotMotor.getVelocity().getValueAsDouble();
        inputs.supplyCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.torqueCurrent = pivotMotor.getTorqueCurrent().getValueAsDouble();
        inputs.temperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.appliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    }
}
