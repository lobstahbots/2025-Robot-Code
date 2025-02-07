package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CHANNEL);

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