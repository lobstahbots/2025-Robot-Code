package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CHANNEL);

  // private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  // public double currentFilterValue = 0.0;
  // public boolean hasZeroed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public boolean limitSwitch() {
    return !limitSwitch.get();
  }
  
  
  public void setExtension(double position) {
    io.setPosition(position);
    Logger.recordOutput("Elevator/Setpoint", position);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getExtension() {
    return inputs.rightPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    if (limitSwitch()) {
      io.resetEncoder(0);
    }
  }
}