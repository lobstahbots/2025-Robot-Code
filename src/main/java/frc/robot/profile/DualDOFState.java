package frc.robot.profile;

/**
 * A record to store the state (position/velocity) of two degrees of freedom
 */
public record DualDOFState(
        /**
         * The position of the first DOF
         */
        double dof1Pos,
        /**
         * The position of the second DOF
         */
        double dof2Pos,
        /**
         * The velocity of the first DOF
         */
        double dof1Vel,
        /**
         * The velocity of the second DOF
         */
        double dof2Vel) {
    /**
     * Get this as a position state, ignoring velocities
     * 
     * @return just the position state
     */
    public DualDOFPositionState getPositionState() {
        return new DualDOFPositionState(dof1Pos, dof2Pos);
    }
}
