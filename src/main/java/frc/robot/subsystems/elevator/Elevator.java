package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CHANNEL);

  private final Mechanism2d mechanism = new Mechanism2d(RobotConstants.TRACK_WIDTH + Units.feetToMeters(3),
      ElevatorConstants.TOP_HEIGHT + Units.feetToMeters(3));
  private final MechanismRoot2d root = mechanism.getRoot("superstructure",
      RobotConstants.TRACK_WIDTH / 2 + Units.feetToMeters(1.5), 0);
  private final MechanismLigament2d elevatorLigament = root
      .append(new MechanismLigament2d("elevator", getExtension(), 90));

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
    elevatorLigament.setLength(getExtension());
    SmartDashboard.putData("Superstructure", mechanism);
    if (limitSwitch()) { io.resetEncoder(0); }
  }
}