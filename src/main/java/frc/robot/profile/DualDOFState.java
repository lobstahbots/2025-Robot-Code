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
         * The velocity of the first DOF
         */
        double dof1Vel,
        /**
         * The position of the second DOF
         */
        double dof2Pos,
        /**
         * The velocity of the second DOF
         */
        double dof2Vel) {}
