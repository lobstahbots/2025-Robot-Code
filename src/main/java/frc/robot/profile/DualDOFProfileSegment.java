package frc.robot.profile;

/**
 * Represents a segment of a two-DOF motion profile
 */
public interface DualDOFProfileSegment {
    /**
     * Get the duration of this segment
     * 
     * @return How long it takes to execute this segment, in seconds
     */
    public double getDuration();

    /**
     * Get the profile state at a given time after the start of this segment
     * 
     * @param time time to get state at, in seconds
     * @return the {@link DualDOFState} representing the desired state (DOF 1/2
     *         position and velocity)
     */
    public DualDOFState calculate(double time);

    /**
     * Get the initial state in the profile segment
     * 
     * @return the initial state
     */
    public DualDOFState getInitialState();

    /**
     * Get the state when this is done executing
     * 
     * @return the state when this is done executing
     */
    public DualDOFState getFinalState();
}
