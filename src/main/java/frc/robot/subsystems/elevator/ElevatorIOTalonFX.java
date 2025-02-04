package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOTalonFX extends SubsystemBase {

  private final TalonFX leftElevatorMotor;
  private final TalonFX rightElevatorMotor;
//   private final StatusSignal<Angle> rightPosition;
//   private final StatusSignal<AngularVelocity> rightVelocity;
//   private final StatusSignal<Voltage> rightAppliedVoltage;
//   private final StatusSignal<Current> rightSupplyCurrent;
//   private final StatusSignal<Current> rightTorqueCurrent;
//   private final StatusSignal<Temperature> rightTempCelsius;
//   private final StatusSignal<Angle> leftPosition;
//   private final StatusSignal<AngularVelocity> leftVelocity;
//   private final StatusSignal<Voltage> leftAppliedVoltage;
//   private final StatusSignal<Current> leftSupplyCurrent;
//   private final StatusSignal<Current> leftTorqueCurrent;
//   private final StatusSignal<Temperature> leftTempCelsius;
//   private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  // private final PIDController leftElevatorController;
  // private final PIDController rightElevatorController;
//   private final DigitalInput topLimit;
//   private final DigitalInput bottomLimit;
//   private final double holdLeftPosValue;
//   private final double holdRightPosValue;

  public ElevatorIOTalonFX(int leftElevatorID, int rightElevatorID) {

    leftElevatorMotor = new TalonFX(leftElevatorID);
    rightElevatorMotor = new TalonFX(rightElevatorID);

    // topLimit = new DigitalInput(0);
    // bottomLimit = new DigitalInput(1);
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftElevatorMotor.getConfigurator().apply(config);
    rightElevatorMotor.getConfigurator().apply(config);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    leftElevatorMotor.getConfigurator().apply(slot0Configs);
    rightElevatorMotor.getConfigurator().apply(slot0Configs);
    
    // rightPosition = rightElevatorMotor.getPosition();
    // rightVelocity = rightElevatorMotor.getVelocity();
    // rightAppliedVoltage = rightElevatorMotor.getMotorVoltage();
    // rightSupplyCurrent = rightElevatorMotor.getSupplyCurrent();
    // rightTorqueCurrent = rightElevatorMotor.getTorqueCurrent();
    // rightTempCelsius = rightElevatorMotor.getDeviceTemp();
    // leftPosition = leftElevatorMotor.getPosition();
    // leftVelocity = leftElevatorMotor.getVelocity();
    // leftAppliedVoltage = leftElevatorMotor.getMotorVoltage();
    // leftSupplyCurrent = leftElevatorMotor.getSupplyCurrent();
    // leftTorqueCurrent = leftElevatorMotor.getTorqueCurrent();
    // leftTempCelsius = leftElevatorMotor.getDeviceTemp();

  }


//   public boolean getBottomLimits() {
//     return !bottomLimit.get();
//   }

//     @Override
//   public void updateInputs(ElevatorIOInputs inputs) {
//     inputs.connected =
//         BaseStatusSignal.refreshAll(
//                 rightPosition, rightVelocity, rightAppliedVoltage, rightSupplyCurrent, rightTorqueCurrent, rightTempCelsius, leftPosition, leftVelocity, leftAppliedVoltage, leftSupplyCurrent, leftTorqueCurrent, leftTempCelsius)
//             .isOK();
//     inputs.rightPositionRads = Units.rotationsToRadians(rightPosition.getValueAsDouble());
//     inputs.rightVelocityRadsPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
//     inputs.rightAppliedVoltage = rightAppliedVoltage.getValueAsDouble();
//     inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
//     inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
//     inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
//     inputs.leftPositionRads = Units.rotationsToRadians(leftPosition.getValueAsDouble());
//     inputs.leftVelocityRadsPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
//     inputs.leftAppliedVoltage = leftAppliedVoltage.getValueAsDouble();
//     inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
//     inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
//     inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();
//   }


  

//   public void setHoldPos() {

//   }

//   public void runVolts(double volts) {
//     leftElevatorMotor.setControl(voltageOut.withOutput(volts));
//     rightElevatorMotor.setControl(voltageOut.withOutput(volts));

//     }


  public void stop() {
    leftElevatorMotor.setControl(neutralOut);
    rightElevatorMotor.setControl(neutralOut);
    }

  
  public void periodic() {
    // if (getBottomLimits()){
    //     resetEncoders();
    //     holdLeftPosValue = 0;
    //     holdRightPosValue = 0;
    // This method will be called once per scheduler run
  }

}