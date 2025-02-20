package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;

public class Superstructure extends SubsystemBase {

    // private final ElevatorIO elevatorIO;
    // private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

    // public final Trigger limitSwitch = new Trigger(() -> elevatorInputs.limitSwitchHit);

    private final Mechanism2d mechanism = new Mechanism2d(RobotConstants.TRACK_WIDTH + Units.feetToMeters(3),
            ElevatorConstants.TOP_HEIGHT + Units.feetToMeters(3));
    private final MechanismRoot2d root = mechanism.getRoot("superstructure",
            RobotConstants.TRACK_WIDTH / 2 + Units.feetToMeters(1.5), 0);
    private final MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevator", getExtension(), 90));
    private final MechanismLigament2d pivotLigament = elevatorLigament
            .append(new MechanismLigament2d("pivot", PivotConstants.ARM_LENGTH, 90));

    private final ProfiledPIDController armPID = new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI,
            PivotConstants.kD, PivotConstants.CONSTRAINTS);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG,
            PivotConstants.kV, PivotConstants.kA);

    public final Trigger atSetpoint = new Trigger(armPID::atSetpoint);

    public Superstructure(/*ElevatorIO elevatorIO, */ PivotIO pivotIO) {
        // this.elevatorIO = elevatorIO;
        this.pivotIO = pivotIO;
    }

    public void setState(SuperstructureState state) {
        // setExtension(state.elevatorHeight);
        setRotation(state.pivotRotation);
    }

    public Command setStateCommand(SuperstructureState state) {
        return startEnd(() -> setState(state), this::stopMotion);
    }

    public void stopMotion() {
        setState(getState());
        // elevatorIO.stop();
        pivotIO.stop();
    }

    public void setExtension(double height) {
        // elevatorIO.setPosition(height);
    }

    public void setRotation(Rotation2d rotation) {
        armPID.setGoal(rotation.getRotations());
    }

    public SuperstructureState getState() {
        return new SuperstructureState(pivotInputs.position, getExtension());
    }

    public void setVoltage(double voltage) {
        // elevatorIO.setVoltage(voltage);
    }

    public double getExtension() {
        return ElevatorConstants.BOTTOM_HEIGHT;
        // return elevatorInputs.rightPosition;
    }

    @Override
    public void periodic() {
        // elevatorIO.updateInputs(elevatorInputs);
        pivotIO.updateInputs(pivotInputs);
        // Logger.processInputs("Superstructure/Elevator", elevatorInputs);
        Logger.processInputs("Superstructure/Pivot", pivotInputs);
        // if (limitSwitch.getAsBoolean()) elevatorIO.resetEncoder(ElevatorConstants.BOTTOM_HEIGHT);
        elevatorLigament.setLength(getExtension());
        pivotLigament.setAngle(pivotInputs.position);
        SmartDashboard.putData("Superstructure", mechanism);
        pivotIO.setVoltage(armPID.calculate(pivotInputs.position.getRotations())
                + armFeedforward.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity));
    }
}