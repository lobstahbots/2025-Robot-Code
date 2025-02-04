package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private final TalonFX leftElevatorMotor;
  private final TalonFX rightElevatorMotor;
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0); // TODO: Put in Constants

  // private final DigitalInput topLimit;
  // private final DigitalInput bottomLimit;


  public Elevator(int leftElevatorID, int rightElevatorID) {

    leftElevatorMotor = new TalonFX(leftElevatorID);
    rightElevatorMotor = new TalonFX(rightElevatorID);

    // bottomLimit = new DigitalInput(1);
    // topLimit = new DigitalInput(0);
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40; // TODO: Put in Constants
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR_RATIO;

    leftElevatorMotor.getConfigurator().apply(config);
    rightElevatorMotor.getConfigurator().apply(config);

    leftElevatorMotor.setControl(new Follower(rightElevatorID, true));

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 2.4; // TODO: Put in Constants
    slot0Configs.kI = 0; // TODO: Put in Constants
    slot0Configs.kD = 0.1; // TODO: Put in Constants

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = 0.5; // TODO: Put in constants
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.5; // TODO: Put in Constants
    

    rightElevatorMotor.getConfigurator().apply(slot0Configs);
  }

  public void setElevatorSpeed(double speed) {
      leftElevatorMotor.set(speed);
      rightElevatorMotor.set(speed);
  }
    
  // public boolean getBottomLimit() {
  //   return !bottomLimit.get();
  // }

  // public boolean getTopLimit() {
  //   return !topLimit.get();
  // }

  public double getEncoderPos() {
    return rightElevatorMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder() {
    rightElevatorMotor.setPosition(0);
  }

  public void setPosition(double position) {
    rightElevatorMotor.setControl(positionVoltage.withPosition(position));
  }

  public double getExtension() {
    return getEncoderPos() * ElevatorConstants.GEAR_RATIO * ElevatorConstants.PITCH_DIAMETER * Math.PI;
  }

  public void periodic() {
    // if (getBottomLimits()){
    //     resetEncoder();

    // This method will be called once per scheduler run
  }
}