package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.profile.DualDOFState;

public class SuperstructureState {
    /**
     * The rotation of the pivot
     */
    public final Rotation2d pivotRotation;
    /**
     * The height of the elevator in meters
     */
    public final double elevatorHeight;
    /**
     * The velocity of the pivot in rad/s
     */
    public final double pivotVelocity;
    /**
     * The velocity of the elevator in m/s
     */
    public final double elevatorVelocity;

    public SuperstructureState(Rotation2d pivotRotation, double elevatorHeight, double pivotVelocity,
            double elevatorVelocity) {
        this.pivotRotation = pivotRotation;
        this.elevatorHeight = elevatorHeight;
        this.pivotVelocity = pivotVelocity;
        this.elevatorVelocity = elevatorVelocity;
    }

    public DualDOFState toDualDOFState() {
        return new DualDOFState(elevatorHeight, pivotRotation.getRadians(), elevatorVelocity, pivotVelocity);
    }

    public static SuperstructureState fromDualDOFState(DualDOFState state) {
        return new SuperstructureState(Rotation2d.fromRadians(state.dof2Pos()), state.dof1Pos(), state.dof2Vel(), state.dof1Vel());
    }
}