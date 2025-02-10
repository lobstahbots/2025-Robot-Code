package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureState {
    public final Rotation2d pivotRotation;
    public final double elevatorHeight;

    public SuperstructureState(Rotation2d pivotRotation, double elevatorHeight) {
        this.pivotRotation = pivotRotation;
        this.elevatorHeight = elevatorHeight;
    }
}
