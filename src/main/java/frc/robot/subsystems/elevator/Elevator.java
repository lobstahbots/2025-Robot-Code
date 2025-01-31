package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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

public class Elevator extends SubsystemBase {

  private final TalonFX leftElevatorMotor;
  private final TalonFX rightElevatorMotor;
  private final DigitalInput limitSwitch;
  private final PIDController elevatorController;

  // private final DigitalInput topLimit;
  // private final DigitalInput bottomLimit;
//   private final double holdLeftPosValue;
//   private final double holdRightPosValue;

  public Elevator(int leftElevatorID, int rightElevatorID) {

    leftElevatorMotor = new TalonFX(leftElevatorID);
    rightElevatorMotor = new TalonFX(rightElevatorID);

    // bottomLimit = new DigitalInput(1);
    // topLimit = new DigitalInput(0);
    limitSwitch = new DigitalInput(0);
    
    TalonFXConfiguration config = new TalonFXConfiguration();
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

  public double getLeftEncoderPos() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  public double getRightEncoderPos() {
    return rightElevatorMotor.getPosition().getValueAsDouble();
  }

  public void resetRightEncoder() {
    rightElevatorMotor.setPosition(0);
  }

  public void resetLeftEncoder() {
    leftElevatorMotor.setPosition(0);
  }

  public void resetElevatorPID() {

  }

  public double getLength() {
    
  }

  public double getExtension() {

  }
//   public void setHoldPos() {

//   }


  public void periodic() {
    // if (getBottomLimits()){
    //     resetEncoders();
    //     holdLeftPosValue = 0;
    //     holdRightPosValue = 0;
    // This method will be called once per scheduler run
  }
}