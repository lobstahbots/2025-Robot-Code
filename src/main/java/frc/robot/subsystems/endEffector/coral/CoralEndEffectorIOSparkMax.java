package frc.robot.subsystems.endEffector.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CoralEndEffectorConstants;

public class CoralEndEffectorIOSparkMax implements CoralEndEffectorIO {
    private final SparkMax leftMotor;
    //private final SparkMax rightMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput beamBreak;

    public CoralEndEffectorIOSparkMax(int leftId, int beamBreakId) {
        leftMotor = new SparkMax(leftId, MotorType.kBrushless);
        //rightMotor = new SparkMax(rightId, MotorType.kBrushless);
        beamBreak = new DigitalInput(beamBreakId);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(CoralEndEffectorConstants.CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.encoder.velocityConversionFactor(1.0);
        //rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // rightMotor.configure(config.follow(leftId, true), ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leftMotor.getEncoder();
    }

    @Override
    public void stopMotor() {
        leftMotor.stopMotor();
    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    public void setSpeed(double speed) {
        leftMotor.set(speed);
    }

    @Override
    public void updateInputs(CoralEndEffectorIOInputs inputs) {
        inputs.velocity = encoder.getVelocity();
        inputs.appliedVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.currentAmps = leftMotor.getOutputCurrent();
        inputs.tempCelsius = leftMotor.getMotorTemperature();
        inputs.beamBreakTriggered = !beamBreak.get();
    }

    @Override
    public void setIdleMode(boolean isBrake) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}