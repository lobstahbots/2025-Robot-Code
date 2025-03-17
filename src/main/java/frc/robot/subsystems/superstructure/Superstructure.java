package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.superstructure.ElevatorPositionCommand;
import frc.robot.commands.superstructure.PivotPositionCommand;
import frc.robot.commands.superstructure.SuperstructureStateCommand;
import frc.robot.util.sysId.CharacterizableSubsystem;

public class Superstructure extends CharacterizableSubsystem {

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
    private final PIDController armVelocityPID = new PIDController(PivotConstants.VELOCITY_kP,
            PivotConstants.VELOCITY_kI, PivotConstants.VELOCITY_kD);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotConstants.kS, 0, PivotConstants.kV,
            PivotConstants.kA);

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.kP,
            ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.CONSTRAINTS);
    private final PIDController elevatorVelocityPID = new PIDController(ElevatorConstants.VELOCITY_kP,
            ElevatorConstants.VELOCITY_kI, ElevatorConstants.VELOCITY_kD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,
            ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    private boolean pivotIsClosedLoop = true;
    private boolean elevatorIsClosedLoop = true;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public Superstructure(ElevatorIO elevatorIO, PivotIO pivotIO) {
        this.elevatorIO = elevatorIO;
        this.pivotIO = pivotIO;
        armPID.setTolerance(0.2);
        elevatorPID.setTolerance(0.05);
        setState(RobotConstants.INTAKE_STATE);
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
        elevatorPID.setGoal(new TrapezoidProfile.State(height, velocity));
        elevatorIsClosedLoop = true;
    }

    public void setRotation(Rotation2d rotation) {
        armPID.setGoal(rotation.getRadians());
        pivotIsClosedLoop = true;
    }

    public SuperstructureState getState() {
        return new SuperstructureState(pivotInputs.position, getExtension(), pivotInputs.velocity,
                elevatorInputs.rightVelocity);
    }

    public SuperstructureState getGoal() {
        return new SuperstructureState(Rotation2d.fromRadians(armPID.getGoal().position),
                elevatorPID.getGoal().position, armPID.getGoal().velocity, elevatorPID.getGoal().velocity);
    }

    public void setElevatorVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public void runVolts(double voltage) {
        setElevatorVoltage(voltage);
    }

    public void setPivotVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
        pivotIsClosedLoop = false;
    }

    public double getExtension() {
        return elevatorInputs.leftPosition;
    }

    public Rotation2d getRotation() {
        return pivotInputs.position;
    }

    public void reset(SuperstructureState state) {
        armPID.reset(state.pivotRotation.getRadians());
        elevatorPID.reset(state.elevatorHeight);
    }

    public void resetRotations(SuperstructureState state) {
        armPID.reset(state.pivotRotation.getRadians());
    }

    public boolean atSetpoint() {
        return atElevatorSetpoint() && atPivotSetpoint();
    }

    public boolean atElevatorSetpoint() {
        return elevatorPID.atGoal();
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
        return this.defer(() -> {
            Command superstructureCommand = new SuperstructureStateCommand(this, setpoint);
            if (setpoint.pivotRotation.getRotations() < PivotConstants.LOWER_DANGER_ZONE.getRotations()
                    && getRotation().getRotations() > PivotConstants.LOWER_DANGER_ZONE.getRotations()) {
                superstructureCommand = new SuperstructureStateCommand(this, new SuperstructureState(
                        PivotConstants.LOWER_DANGER_ZONE, RobotConstants.INTAKE_STATE.elevatorHeight, 0, 0))
                                .until(() -> {
                                    System.out.println(getExtension());
                                    return getExtension() < 1;
                                }).andThen(superstructureCommand);
            }

            if (getRotation().getRotations() < PivotConstants.LOWER_DANGER_ZONE.getRotations()
                    && setpoint.pivotRotation.getRotations() > PivotConstants.LOWER_DANGER_ZONE.getRotations()) {
                return new PivotPositionCommand(this, PivotConstants.LOWER_DANGER_ZONE).andThen(superstructureCommand);
            }

            return superstructureCommand;
        });
    }

    public Rotation2d getPivotRotation() {
        return pivotInputs.position;
    }

    public void setIdleMode(boolean isBrake) {
        elevatorIO.setIdleMode(isBrake);
        pivotIO.setIdleMode(isBrake);
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Superstructure/Elevator", elevatorInputs);
        Logger.processInputs("Superstructure/Pivot", pivotInputs);
        if (limitSwitch.getAsBoolean()) elevatorIO.resetEncoder(ElevatorConstants.BOTTOM_HEIGHT);
        elevatorLigament.setLength(getExtension() / 100);
        pivotLigament.setAngle(getRotation());
        SmartDashboard.putData("Superstructure", mechanism);
        Logger.recordOutput("Superstructure", pivotInputs.position);
        Logger.recordOutput("Goal", armPID.getGoal().position);
        Logger.recordOutput("Setpoint", armPID.getSetpoint().position);
        SmartDashboard.putData("pivotPID", armPID);

        Logger.recordOutput("ElevatorPOsition", elevatorInputs.leftPosition);
        Logger.recordOutput("ElevatorGoal", elevatorPID.getGoal().position);
        Logger.recordOutput("ElevatorSetpoint", elevatorPID.getSetpoint().position);
        SmartDashboard.putData("ElevatorPID", elevatorPID);
        Logger.recordOutput("PivotAtGoal", atPivotSetpoint());
        Logger.recordOutput("ElevatorAtGoal", atElevatorSetpoint());
        Logger.recordOutput("ElevatorVelocitySetpoint", elevatorPID.getSetpoint().velocity);
        SmartDashboard.putData("ElevatorVelocityPID", elevatorVelocityPID);
        SmartDashboard.putData("PivotVelocityPID", armVelocityPID);
        Logger.recordOutput("PivotVelocitySetpoint", armPID.getSetpoint().velocity);

        if (pivotIsClosedLoop) pivotIO.setVoltage(armPID.calculate(pivotInputs.position.getRadians())
                + armFeedforward.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity)
                + armVelocityPID.calculate(getRotation().getRadians(), armPID.getSetpoint().velocity)
                + Math.cos(pivotInputs.position.plus(PivotConstants.COG_OFFSET).getRadians()) * PivotConstants.kG);

        if (elevatorIsClosedLoop) setElevatorVoltage(elevatorPID.calculate(elevatorInputs.leftPosition)
                + elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity)
                + elevatorVelocityPID.calculate(elevatorInputs.leftVelocity, elevatorPID.getSetpoint().velocity));

    }
}
