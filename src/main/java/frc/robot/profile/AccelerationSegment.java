package frc.robot.profile;

/**
 * A profile segment consisting of accelerating to certain speeds
 */
public class AccelerationSegment implements DualDOFProfileSegment {
    private final DualDOFState initialState;
    private final double duration;
    private final double dof1Acc;
    private final double dof2Acc;

    /**
     * Create an acceleration segment. Note that while this segment will work to
     * accelerate to velocity within acceleration limits even if the acceleration
     * limits are not in the same proportion as the velocity limits, it will simply
     * not accelerate as fast on the DOF which can accelerate to velocity sooner;
     * this means that the velocity ratio between the two DOFs will be maintained
     * throughout the whole profile segment, if you start with zero velocity.
     * 
     * Also note that while this segment is called an acceleration segment it will
     * function to decelerate as well; you can change from any state to any two
     * speeds. Either way, while velocities can be negative, acceleration limits
     * must be positive and will be treated as limits on the absolute value of the
     * acceleration of the mechanism.
     * 
     * @param initialState the initial state of the 2-DOF mechanism
     * @param dof1Vel      the final velocity of the first DOF
     * @param dof2Vel      the final velocity of the second DOF
     * @param dof1AccMax   the maximum acceleration of the first DOF; must be
     *                     positive
     * @param dof2AccMax   the maximum accleration of the second DOF; must be
     *                     positive
     */
    public AccelerationSegment(DualDOFState initialState, double dof1Vel, double dof2Vel, double dof1AccMax,
            double dof2AccMax) {
        this.initialState = initialState;
        duration = Math.max(Math.abs(dof1Vel - initialState.dof1Vel()) / dof1AccMax,
                Math.abs(dof2Vel - initialState.dof2Vel()) / dof2AccMax);
        this.dof1Acc = (dof1Vel - initialState.dof1Vel()) / duration;
        this.dof2Acc = (dof2Vel - initialState.dof2Vel()) / duration;
    }

    public double getDuration() {
        return duration;
    }

    /**
     * Get the state when this is done executing
     * 
     * @return the state when this is done executing
     */
    public DualDOFState getFinalState() {
        return calculate(duration);
    }

    public DualDOFState calculate(double time) {
        return new DualDOFState(initialState.dof1Pos() + time * initialState.dof1Vel() + time * time * dof1Acc / 2,
                initialState.dof1Vel() + time * dof1Acc,
                initialState.dof2Pos() + time * initialState.dof2Vel() + time * time * dof2Acc / 2,
                initialState.dof2Vel() + time * dof2Acc);
    }
}
