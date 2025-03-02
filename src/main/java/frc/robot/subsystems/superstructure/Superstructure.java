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
import frc.robot.commands.superstructure.ElevatorPositionCommand;
import frc.robot.commands.superstructure.PivotPositionCommand;
import frc.robot.commands.superstructure.SuperstructureStateCommand;

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

    private final ProfiledPIDController armPID = new ProfiledPIDController(PivotConstants.kP, PivotConstants.kI,
            PivotConstants.kD, PivotConstants.CONSTRAINTS);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotConstants.kS, PivotConstants.kG,
            PivotConstants.kV, PivotConstants.kA);

    private boolean pivotIsClosedLoop = true;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public Superstructure(ElevatorIO elevatorIO, PivotIO pivotIO) {
        this.elevatorIO = elevatorIO;
        this.pivotIO = pivotIO;
    }

    public void setState(SuperstructureState state) {
        setExtension(state.elevatorHeight, state.elevatorVelocity);
        setRotation(state.pivotRotation, state.pivotVelocity);
    }

    public void setRotation(Rotation2d rotation, double velocity) {
        armPID.setGoal(new TrapezoidProfile.State(rotation.getRadians(), velocity));
        pivotIsClosedLoop = true;
    }

    public void stopMotion() {
        setState(getState());
        elevatorIO.stop();
        pivotIO.stop();
    }

    public void setExtension(double height, double velocity) {
        elevatorIO.setPosition(new TrapezoidProfile.State(height, velocity));
    }

    public void setRotation(Rotation2d rotation) {
        armPID.setGoal(rotation.getRotations());
    }

    public SuperstructureState getState() {
        return new SuperstructureState(pivotInputs.position, getExtension(), pivotInputs.velocity,
                elevatorInputs.rightVelocity);
    }

    public void setElevatorVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public void setPivotVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
        pivotIsClosedLoop = false;
    }

    public double getExtension() {
        return elevatorInputs.rightPosition;
    }

    public Rotation2d getRotation() {
        return pivotInputs.position;
    }

    public void reset(SuperstructureState state) {
        armPID.reset(state.pivotRotation.getRotations());
        elevatorIO.resetEncoder(state.elevatorHeight);
    }

    public boolean atSetpoint() {
        return atElevatorSetpoint() && atPivotSetpoint();
    }

    public boolean atElevatorSetpoint() {
        return elevatorInputs.atSetpoint;
    }

    public boolean atPivotSetpoint() {
        return armPID.atGoal();
    }

    /**
     * Returns a command that moves the superstructure to the given setpoint with
     * avoidance of danger zones.
     * 
     * @param setpoint The setpoint to move to.
     */
    public Command getSetpointCommand(SuperstructureState setpoint) {
        Command superstructureCommand = new SuperstructureStateCommand(this, setpoint);

        if (setpoint == RobotConstants.INTAKE_STATE) {
            superstructureCommand = new ElevatorPositionCommand(this, RobotConstants.INTAKE_STATE.elevatorHeight)
                    .andThen(superstructureCommand);
        }

        if (getRotation().getRotations() > PivotConstants.LOWER_DANGER_ZONE.getRotations()) {
            return new PivotPositionCommand(this, PivotConstants.LOWER_DANGER_ZONE).andThen(superstructureCommand);
        } else if (getRotation().getRotations() < PivotConstants.UPPER_DANGER_ZONE.getRotations()) {
            return new PivotPositionCommand(this, PivotConstants.UPPER_DANGER_ZONE).andThen(superstructureCommand);
        }

        return superstructureCommand;

    }

    public Rotation2d getPivotRotation() {
        return pivotInputs.position;
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
        Logger.recordOutput("Superstructure", pivotInputs.position);
        Logger.recordOutput("Setpoint", armPID.getSetpoint().toString());

        if (pivotIsClosedLoop) pivotIO.setVoltage(armPID.calculate(pivotInputs.position.getRadians())
                + armFeedforward.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity));
    }
}