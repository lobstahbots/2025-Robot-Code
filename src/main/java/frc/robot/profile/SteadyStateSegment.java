package frc.robot.profile;

/**
 * Represents a segment at steady state - that is, both
 */
public class SteadyStateSegment implements DualDOFProfileSegment {
    private final DualDOFPositionState initialState;
    private final double dof1Vel;
    private final double dof2Vel;
    private final double duration;

    /**
     * Create a steady state segment
     * 
     * @param initialState the position that the things start at, they are assumed
     *                     to be in the velocities specified by the other parameters
     * @param dof1Vel      velocity of the first DOF
     * @param dof2Vel      velocity of the second DOF
     * @param duration     how long this will execute for
     */
    public SteadyStateSegment(DualDOFPositionState initialState, double dof1Vel, double dof2Vel, double duration) {
        this.initialState = initialState;
        this.dof1Vel = dof1Vel;
        this.dof2Vel = dof2Vel;
        this.duration = duration;
    }

    /**
     * Create a steady state segment
     * 
     * @param initialState the state that things start at. the position is where
     *                     they start and the velocities of this state are what the
     *                     segment assumes to be the steady state so they will
     *                     remain the same
     * @param duration
     */
    public SteadyStateSegment(DualDOFState initialState, double duration) {
        this(initialState.getPositionState(), initialState.dof1Vel(), initialState.dof2Vel(), duration);
    }

    /**
     * Initialize a steady state segment from an initial state and a final position
     * state. Note that it is assumed that the velocities of the initial state will
     * eventually go to the final state; that is they are in the direction of the
     * final state.
     * 
     * @param initialState The initial state, including velocity, which will be held
     *                     for the duration of this segment
     * @param finalState   The position where it should end up at. The position for
     *                     DOF 2 is ignored and only the position for DOF 1 is used
     *                     to calculate how long this profile should take, unless
     *                     the velocity for DOF 1 is zero, in which case only the
     *                     second DOF is used.
     */
    public SteadyStateSegment(DualDOFState initialState, DualDOFPositionState finalState) {
        this(initialState,
                initialState.dof1Vel() != 0 ? (finalState.dof1Pos() - initialState.dof1Pos()) / initialState.dof1Vel()
                        : (finalState.dof2Pos() - initialState.dof2Pos()) / initialState.dof2Vel());
    }

    /**
     * See {@link #SteadyStateSegment(DualDOFState, DualDOFPositionState) other
     * constructor}; this is equivalent except velocity of the final state is
     * ignored.
     * 
     * @param initialState The initial state, including velocity, which will be held
     *                     for the duration of this segment
     * @param finalState   The position where it should end up at. The position for
     *                     DOF 2 is ignored and only the position for DOF 1 is used
     *                     to calculate how long this profile should take.
     *                     velocities here are ignored altogether
     */
    public SteadyStateSegment(DualDOFState initialState, DualDOFState finalState) {
        this(initialState, finalState.getPositionState());
    }

    public double getDuration() {
        return duration;
    }

    public DualDOFState calculate(double time) {
        return new DualDOFState(initialState.dof1Pos() + dof1Vel * time, initialState.dof2Pos() + dof2Vel * time,
                dof1Vel, dof2Vel);
    }

    public DualDOFState getInitialState() {
        return new DualDOFState(initialState.dof1Pos(), initialState.dof2Pos(), dof1Vel, dof2Vel);
    }

    public DualDOFState getFinalState() {
        return calculate(duration);
    }
}
