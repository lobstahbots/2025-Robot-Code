package frc.robot.subsystems.superstructure;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.profile.DualDOFPositionState;
import frc.robot.profile.DualDOFProfile;
import frc.robot.util.sysId.CharacterizableSubsystem;

public class Superstructure extends CharacterizableSubsystem {
    private SuperstructureState currentSetpoint;
    private SuperstructureState goal;

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

    private final PIDController armPID = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
    private final PIDController armVelocityPID = new PIDController(PivotConstants.VELOCITY_kP,
            PivotConstants.VELOCITY_kI, PivotConstants.VELOCITY_kD);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(PivotConstants.kS, 0, PivotConstants.kV,
            PivotConstants.kA);

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI,
            ElevatorConstants.kD);
    private final PIDController elevatorVelocityPID = new PIDController(ElevatorConstants.VELOCITY_kP,
            ElevatorConstants.VELOCITY_kI, ElevatorConstants.VELOCITY_kD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS,
            ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    private boolean closedLoop = true;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public Superstructure(ElevatorIO elevatorIO, PivotIO pivotIO) {
        this.elevatorIO = elevatorIO;
        this.pivotIO = pivotIO;
        armPID.setTolerance(0.2);
        elevatorPID.setTolerance(0.05);
        setState(RobotConstants.INTAKE_STATE);
    }

    public void setState(SuperstructureState state) {
        closedLoop = true;
        currentSetpoint = state;
        goal = null;
    }

    public void stopMotion() {
        setState(getState());
        elevatorIO.stop();
        pivotIO.stop();
    }

    public SuperstructureState getState() {
        return new SuperstructureState(pivotInputs.position, getExtension(), pivotInputs.velocity,
                elevatorInputs.rightVelocity);
    }

    public SuperstructureState getGoal() {
        return goal;
    }

    public void setElevatorVoltage(double voltage) {
        elevatorIO.setVoltage(voltage);
    }

    public void runVolts(double voltage) {
        setElevatorVoltage(voltage);
    }

    public void setPivotVoltage(double voltage) {
        pivotIO.setVoltage(voltage);
        closedLoop = false;
    }

    public double getExtension() {
        return elevatorInputs.leftPosition;
    }

    public Rotation2d getRotation() {
        return pivotInputs.position;
    }

    public void reset() {
        armPID.reset();
        elevatorPID.reset();
        armVelocityPID.reset();
        elevatorVelocityPID.reset();
    }

    public boolean atSetpoint() {
        return atElevatorSetpoint() && atPivotSetpoint();
    }

    public boolean atElevatorSetpoint() {
        if (goal == null) return elevatorPID.atSetpoint();
        return Math.abs(getExtension() - goal.elevatorHeight) < 0.05;
    }

    public boolean atPivotSetpoint() {
        if (goal == null) return elevatorPID.atSetpoint();
        return Math.abs(getRotation().minus(goal.pivotRotation).getRadians()) < 0.2;
    }

    /**
     * Returns a command that moves the superstructure to the given setpoint with
     * avoidance of danger zones.
     * 
     * @param setpoint The setpoint to move to.
     */
    public Command getSetpointCommand(SuperstructureState setpoint) {
        return this.defer(() -> {
            final Timer timer = new Timer();
            DualDOFProfile profile;
            if ((setpoint.pivotRotation.getRotations() < PivotConstants.LOWER_DANGER_ZONE.getRotations()
                    && getRotation().getRotations() > PivotConstants.LOWER_DANGER_ZONE.getRotations())
                    || (getRotation().getRotations() < PivotConstants.LOWER_DANGER_ZONE.getRotations()
                            && setpoint.pivotRotation.getRotations() > PivotConstants.LOWER_DANGER_ZONE
                                    .getRotations())) {
                profile = DualDOFProfile.fromWaypoints(
                        List.of(getState().toDualDOFState().getPositionState(),
                                new DualDOFPositionState(0, PivotConstants.LOWER_DANGER_ZONE.getRadians()),
                                setpoint.toDualDOFState().getPositionState()),
                        ElevatorConstants.CONSTRAINTS, PivotConstants.CONSTRAINTS);
            } else {
                profile = DualDOFProfile.fromWaypoints(
                        List.of(getState().toDualDOFState().getPositionState(),
                                setpoint.toDualDOFState().getPositionState()),
                        ElevatorConstants.CONSTRAINTS, PivotConstants.CONSTRAINTS);
            }

            return startRun(() -> {
                goal = setpoint;
                closedLoop = true;
                timer.restart();
            }, () -> {
                currentSetpoint = SuperstructureState.fromDualDOFState(profile.calculate(timer.get()));
            });
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
        Logger.recordOutput("Goal", goal.pivotRotation);
        Logger.recordOutput("Setpoint", currentSetpoint.pivotRotation);
        SmartDashboard.putData("pivotPID", armPID);

        Logger.recordOutput("ElevatorPosition", elevatorInputs.leftPosition);
        Logger.recordOutput("ElevatorGoal", goal.elevatorHeight);
        Logger.recordOutput("ElevatorSetpoint", currentSetpoint.elevatorHeight);
        SmartDashboard.putData("ElevatorPID", elevatorPID);
        Logger.recordOutput("PivotAtGoal", atPivotSetpoint());
        Logger.recordOutput("ElevatorAtGoal", atElevatorSetpoint());
        Logger.recordOutput("ElevatorVelocitySetpoint", currentSetpoint.elevatorVelocity);
        SmartDashboard.putData("ElevatorVelocityPID", elevatorVelocityPID);
        SmartDashboard.putData("PivotVelocityPID", armVelocityPID);
        Logger.recordOutput("PivotVelocitySetpoint", currentSetpoint.pivotVelocity);

        double elevatorHeight = elevatorInputs.leftPosition * 0.4 / RobotConstants.L3_STATE.elevatorHeight;
        Logger.recordOutput("ComponentPoses", new Pose3d[] {
                new Pose3d(0.1906, 0, 0.121 + elevatorHeight / 2, Rotation3d.kZero),
                new Pose3d(0.1906, 0, 0.1464 + elevatorHeight, Rotation3d.kZero),
                new Pose3d(0.1906, 0, 0.614 + elevatorHeight,
                        new Rotation3d(0, Units.degreesToRadians(-67) - pivotInputs.position.getRadians(), 0)) });

        if (closedLoop) {
            pivotIO.setVoltage(
                    armPID.calculate(pivotInputs.position.getRadians(), currentSetpoint.pivotRotation.getRadians())
                            + armFeedforward.calculate(currentSetpoint.pivotRotation.getRadians(),
                                    currentSetpoint.pivotVelocity)
                            + armVelocityPID.calculate(pivotInputs.velocity, currentSetpoint.pivotVelocity)
                            + Math.cos(pivotInputs.position.plus(PivotConstants.COG_OFFSET).getRadians())
                                    * PivotConstants.kG);

            setElevatorVoltage(elevatorPID.calculate(elevatorInputs.leftPosition, currentSetpoint.elevatorHeight)
                    + elevatorFeedforward.calculate(currentSetpoint.elevatorVelocity)
                    + elevatorVelocityPID.calculate(elevatorInputs.leftVelocity, currentSetpoint.elevatorVelocity));
        }
    }
}
