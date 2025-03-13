package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX leftElevatorMotor;
    private final TalonFX rightElevatorMotor;

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightAppliedVoltage;
    private final StatusSignal<Current> rightSupplyCurrent;
    private final StatusSignal<Current> rightStatorCurrent;
    private final StatusSignal<Current> rightTorqueCurrent;
    private final StatusSignal<Temperature> rightTempCelsius;
    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftAppliedVoltage;
    private final StatusSignal<Current> leftSupplyCurrent;
    private final StatusSignal<Current> leftStatorCurrent;
    private final StatusSignal<Current> leftTorqueCurrent;
    private final StatusSignal<Temperature> leftTempCelsius;

    private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CHANNEL);

    public ElevatorIOTalonFX(int leftElevatorID, int rightElevatorID) {
        leftElevatorMotor = new TalonFX(leftElevatorID);
        rightElevatorMotor = new TalonFX(rightElevatorID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //change before match
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER
                * Math.PI;

        leftElevatorMotor.getConfigurator().apply(config);
        rightElevatorMotor.getConfigurator().apply(config);

        leftElevatorMotor.setControl(new Follower(rightElevatorID, false)); // it should be true because they are opposed but phoenix 6 a[[ears to do the opposite of what we tell it to do

        rightPosition = rightElevatorMotor.getPosition();
        rightVelocity = rightElevatorMotor.getVelocity();
        rightAppliedVoltage = rightElevatorMotor.getMotorVoltage();
        rightSupplyCurrent = rightElevatorMotor.getSupplyCurrent();
        rightStatorCurrent = rightElevatorMotor.getStatorCurrent();
        rightTorqueCurrent = rightElevatorMotor.getTorqueCurrent();
        rightTempCelsius = rightElevatorMotor.getDeviceTemp();
        leftPosition = leftElevatorMotor.getPosition();
        leftVelocity = leftElevatorMotor.getVelocity();
        leftAppliedVoltage = leftElevatorMotor.getMotorVoltage();
        leftSupplyCurrent = leftElevatorMotor.getSupplyCurrent();
        leftStatorCurrent = leftElevatorMotor.getStatorCurrent();
        leftTorqueCurrent = leftElevatorMotor.getTorqueCurrent();
        leftTempCelsius = leftElevatorMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(ElevatorConstants.BASE_STATUS_SIGNAL_FREQUENCY, rightPosition,
                rightVelocity, rightAppliedVoltage, rightSupplyCurrent, rightTorqueCurrent, rightTempCelsius,
                leftPosition, leftVelocity, leftAppliedVoltage, leftSupplyCurrent, leftTorqueCurrent, leftTempCelsius);
        rightElevatorMotor.optimizeBusUtilization();
        leftElevatorMotor.optimizeBusUtilization();

        leftElevatorMotor.setPosition(0.0);
        rightElevatorMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightAppliedVoltage, rightSupplyCurrent,
                rightStatorCurrent, rightTorqueCurrent, rightTempCelsius, leftPosition, leftVelocity,
                leftAppliedVoltage, leftSupplyCurrent, leftStatorCurrent, leftTorqueCurrent, leftTempCelsius);
        inputs.rightPosition = rightPosition.getValueAsDouble();
        inputs.rightVelocity = rightVelocity.getValueAsDouble();
        inputs.rightAppliedVoltage = rightAppliedVoltage.getValueAsDouble();
        inputs.rightSupplyCurrent = rightSupplyCurrent.getValueAsDouble();
        inputs.rightStatorCurrent = rightStatorCurrent.getValueAsDouble();
        inputs.rightTorqueCurrent = rightTorqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
        inputs.leftPosition = leftPosition.getValueAsDouble();
        inputs.leftVelocity = leftVelocity.getValueAsDouble();
        inputs.leftAppliedVoltage = leftAppliedVoltage.getValueAsDouble();
        inputs.leftSupplyCurrent = leftSupplyCurrent.getValueAsDouble();
        inputs.leftStatorCurrent = leftStatorCurrent.getValueAsDouble();
        inputs.leftTorqueCurrent = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();
        inputs.limitSwitchHit = !limitSwitch.get();
        inputs.atSetpoint = MathUtil.applyDeadband(rightElevatorMotor.getClosedLoopError().getValueAsDouble(),
                ElevatorConstants.HEIGHT_DEADBAND) == 0;
    }

    @Override
    public void setVoltage(double voltage) {
        rightElevatorMotor.setVoltage(voltage);
    }

    @Override
    public void resetEncoder(double position) {
        rightElevatorMotor.setPosition(position);
    }

    @Override
    public void stop() {
        rightElevatorMotor.stopMotor();
    }

    @Override
    public void setIdleMode(boolean isBrake) {
        leftElevatorMotor.setNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}