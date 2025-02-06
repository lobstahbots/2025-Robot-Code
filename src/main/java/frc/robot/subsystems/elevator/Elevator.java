package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  // private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  // public double currentFilterValue = 0.0;
  // public boolean hasZeroed = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  // public Command runCurrentZeroing() {
  //   return this.run(
  //           () -> {
  //             io.setVoltage(-0.5);
  //             Logger.recordOutput("Elevator/Setpoint", Double.NaN);
  //           })
  //       .until(() -> currentFilterValue > 20.0)
  //       .finallyDo(
  //           (interrupted) -> {
  //             if (!interrupted) {
  //               io.resetEncoder(0.0);
  //               hasZeroed = true;
  //             }
  //           });
  // }
  
  public void setExtension(double position) {
    io.setPosition(position);
    Logger.recordOutput("Elevator/Setpoint", position);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getExtension() {
    return inputs.rightPosition * ElevatorConstants.PITCH_DIAMETER * Math.PI;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs); // TODO: Is Loggable Inputs the right type?
    // currentFilterValue = currentFilter.calculate(inputs.rightStatorCurrent);
  }
}