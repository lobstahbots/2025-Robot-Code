package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureState {
    public final Rotation2d pivotRotation;
    public final double elevatorHeight;
    public final double pivotVelocity;
    public final double elevatorVelocity;

    public SuperstructureState(Rotation2d pivotRotation, double elevatorHeight, double pivotVelocity, double elevatorVelocity) {
        this.pivotRotation = pivotRotation;
        this.elevatorHeight = elevatorHeight;
        this.pivotVelocity = pivotVelocity;
        this.elevatorVelocity = elevatorVelocity;
    }
}
