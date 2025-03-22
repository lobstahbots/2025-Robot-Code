package frc.robot.profile;

/**
 * A record to store the position of two degrees of freedom
 */
public record DualDOFPositionState(
        /**
         * The position of the first DOF
         */
        double dof1Pos,
        /**
         * The position of the second DOF
         */
        double dof2Pos) {}
