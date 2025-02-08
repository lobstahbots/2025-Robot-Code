package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(ElevatorConstants.MOTION_MAGIC_POSITION_VOLTAGE);
  private final VoltageOut voltageOut = new VoltageOut(ElevatorConstants.VOLTAGE_OUTPUT).withEnableFOC(true);


  public ElevatorIOTalonFX(int leftElevatorID, int rightElevatorID) {

    leftElevatorMotor = new TalonFX(leftElevatorID);
    rightElevatorMotor = new TalonFX(rightElevatorID);

    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER * Math.PI;
    config.Slot0.kP = ElevatorConstants.PID_P;
    config.Slot0.kI = ElevatorConstants.PID_I;
    config.Slot0.kD = ElevatorConstants.PID_D;
    config.Slot0.kS = ElevatorConstants.KS;
    config.Slot0.kV = ElevatorConstants.KV;
    config.Slot0.kA = ElevatorConstants.KA;
    config.Slot0.kG = ElevatorConstants.KG;
    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION;
    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    leftElevatorMotor.getConfigurator().apply(config);
    rightElevatorMotor.getConfigurator().apply(config);
    leftElevatorMotor.setControl(new Follower(rightElevatorID, true));

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

    BaseStatusSignal.setUpdateFrequencyForAll(ElevatorConstants.BASE_STATUS_SIGNAL_FREQUENCY, rightPosition, rightVelocity, rightAppliedVoltage, rightSupplyCurrent, rightTorqueCurrent, rightTempCelsius, leftPosition, leftVelocity, leftAppliedVoltage, leftSupplyCurrent, leftTorqueCurrent, leftTempCelsius);
    rightElevatorMotor.optimizeBusUtilization();
    leftElevatorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(rightPosition, rightVelocity, rightAppliedVoltage, rightSupplyCurrent, rightStatorCurrent, rightTorqueCurrent, rightTempCelsius, leftPosition, leftVelocity, leftAppliedVoltage, leftSupplyCurrent, leftStatorCurrent, leftTorqueCurrent, leftTempCelsius);
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
  }

  @Override
  public void setPosition(double position) {
    rightElevatorMotor.setControl(positionVoltage.withPosition(position));
  }

  @Override
  public void setVoltage(double voltage) {
    rightElevatorMotor.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void resetEncoder(double position) {
    rightElevatorMotor.setPosition(0);
  }

  @Override
  public void stop() {
    rightElevatorMotor.stopMotor();
  }
}