package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.commands.superstructureCommands.ElevatorToPositionCommand;
import frc.robot.commands.superstructureCommands.PivotToPositionCommand;
import frc.robot.commands.superstructureCommands.SuperstructureStateCommand;

public class Superstructure extends SubsystemBase {

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

    public final Trigger limitSwitch = new Trigger(() -> elevatorInputs.limitSwitchHit);

    private final Mechanism2d mechanism = new Mechanism2d(RobotConstants.TRACK_WIDTH + Units.feetToMeters(3),
            ElevatorConstants.TOP_HEIGHT + Units.feetToMeters(3));
    private final MechanismRoot2d root = mechanism.getRoot("superstructure",
            RobotConstants.TRACK_WIDTH / 2 + Units.feetToMeters(1.5), 0);
    private final MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevator", getExtension(), 90));
    private final MechanismLigament2d pivotLigament = elevatorLigament
            .append(new MechanismLigament2d("pivot", PivotConstants.ARM_LENGTH, 90));

    private final ProfiledPIDController pivotPID = new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI,
            PivotConstants.kD, PivotConstants.CONSTRAINTS);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG,
            PivotConstants.kV, PivotConstants.kA);

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public Superstructure(ElevatorIO elevatorIO, PivotIO pivotIO) {
        this.elevatorIO = elevatorIO;
        this.pivotIO = pivotIO;
    }

    public void setGoal(SuperstructureState goal) {
        setRotation(goal.pivotRotation, goal.pivotVelocity);
        setExtension(goal.elevatorHeight, goal.elevatorVelocity);
    }

    public void setRotation(Rotation2d rotation, double velocity) {
        pivotPID.setGoal(new TrapezoidProfile.State(rotation.getRotations(), velocity));
    }

    public void setExtension(double height, double velocity) {
        elevatorIO.setPosition(new TrapezoidProfile.State(height, velocity));
    }

    public SuperstructureState getState() {
        return new SuperstructureState(getRotation(), getExtension(), pivotInputs.velocity, elevatorInputs.rightVelocity);
    }

    public double getExtension() {
        return elevatorInputs.rightPosition;
    }

    public Rotation2d getRotation() {
        return pivotInputs.position;
    }

    public void stopMotion() {
        setGoal(getState());
        elevatorIO.stop();
        pivotIO.stop();
    }

    public void reset(SuperstructureState state) {
        pivotPID.reset(state.pivotRotation.getRotations());
        elevatorIO.resetEncoder(state.elevatorHeight);
    }

    public void setElevatorVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public void setPivotVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
    }

    public boolean atSetpoint() {
        return atElevatorSetpoint() && atPivotSetpoint();
    }

    public boolean atElevatorSetpoint() {
        return elevatorInputs.atSetpoint;
    }

    public boolean atPivotSetpoint() {
        return pivotPID.atGoal();
    }

    /**
     * Returns a command that moves the superstructure to the given setpoint with avoidance of danger zones.
     * @param setpoint The setpoint to move to.
     */
    public Command getSetpointCommand(SuperstructureState setpoint) {
        Command superstructureCommand = new SuperstructureStateCommand(this, setpoint);

        if(setpoint == RobotConstants.INTAKE_STATE) {
            superstructureCommand = new ElevatorToPositionCommand(this, RobotConstants.INTAKE_STATE.elevatorHeight).andThen(superstructureCommand);
        }

        if(getRotation().getRotations() > PivotConstants.LOWER_DANGER_ZONE.getRotations()) {
            return new PivotToPositionCommand(this, PivotConstants.LOWER_DANGER_ZONE).andThen(superstructureCommand);
        } else if (getRotation().getRotations() < PivotConstants.UPPER_DANGER_ZONE.getRotations()) {
            return new PivotToPositionCommand(this, PivotConstants.UPPER_DANGER_ZONE).andThen(superstructureCommand);
        }

        return superstructureCommand;

    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Superstructure/Elevator", elevatorInputs);
        Logger.processInputs("Superstructure/Pivot", pivotInputs);
        if (limitSwitch.getAsBoolean()) elevatorIO.resetEncoder(ElevatorConstants.BOTTOM_HEIGHT);
        elevatorLigament.setLength(getExtension());
        pivotLigament.setAngle(getRotation());
        SmartDashboard.putData("Superstructure", mechanism);
        
        setPivotVoltage(pivotPID.calculate(pivotInputs.position.getRotations())
            + armFeedforward.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity));
    }
}