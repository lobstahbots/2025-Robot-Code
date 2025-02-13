// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.PivotConstants;

public class PivotIOTalonFX implements PivotIO {

    private final TalonFX pivotMotor;
    private final PIDController pivotPID = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward feedForward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG,
            PivotConstants.kV, PivotConstants.kA);

    /** Creates a new Pivot. */
    public PivotIOTalonFX(int pivotMotorID, int encoderChannel) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        pivotMotor = new TalonFX(pivotMotorID);
        config.CurrentLimits.SupplyCurrentLimit = PivotConstants.CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().apply(config);
        encoder = new DutyCycleEncoder(encoderChannel);
    }

    public void setDesiredAngle(double desiredAngle) {
        double pidOutput = pivotPID.calculate(encoder.get(), desiredAngle);
        double feedForwardOutput = feedForward.calculate(encoder.get(), pivotMotor.getPosition().getValueAsDouble());
        pivotMotor.setVoltage(feedForwardOutput + pidOutput);
    }

    public void resetControllerError() {
        pivotPID.reset();
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
        inputs.position = new Rotation2d(pivotMotor.getPosition().getValue());
        inputs.velocity = pivotMotor.getVelocity().getValueAsDouble();
        inputs.supplyCurrent = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.statorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.torqueCurrent = pivotMotor.getTorqueCurrent().getValueAsDouble();
        inputs.temperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.appliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    }
}
