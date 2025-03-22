package frc.robot.profile;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N2;

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
     * @param dof2AccMax   the maximum acceleration of the second DOF; must be
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

    private AccelerationSegment(DualDOFState initialState, double duration, double dof1Acc, double dof2Acc) {
        this.initialState = initialState;
        this.duration = duration;
        this.dof1Acc = dof1Acc;
        this.dof2Acc = dof2Acc;
    }

    /**
     * Create an acceleration segment which will accelerate from a specified
     * position to specified velocities with specified (positive) acceleration
     * limits. That is, specified velocities can be negative, but specified
     * acceleration limits are treated as limits on the absolute value of
     * acceleration.
     * 
     * @param accelerateFrom the position you start at
     * @param dof1Vel        the velocity to accelerate the first DOF to
     * @param dof2Vel        the velocity to accelerate the second DOF to
     * @param dof1AccMax     the maximum acceleration of the first DOF
     * @param dof2AccMax     the maximum acceleration of the second DOF
     * @return the created acceleration segment
     */
    public static AccelerationSegment getAccelerationSegment(DualDOFPositionState accelerateFrom, double dof1Vel,
            double dof2Vel, double dof1AccMax, double dof2AccMax) {
        return new AccelerationSegment(new DualDOFState(accelerateFrom.dof1Pos(), accelerateFrom.dof2Pos(), 0, 0), dof1Vel,
                dof2Vel, dof1AccMax, dof2AccMax);
    }

    /**
     * Create an acceleration segment which will decelerate to a specified position
     * from specified velocities with specified (positive) acceleration limits. That
     * is, specified velocities can be negative, but specified acceleration limits
     * are treated as limits on the absolute value of acceleration.
     * 
     * @param decelerateTo the position you want to end up at
     * @param dof1Vel      the velocity of the first DOF to decelerate from
     * @param dof2Vel      the velocity of the second DOF to decelerate from
     * @param dof1AccMax   the maximum acceleration of the first DOF; must be
     *                     positive
     * @param dof2AccMax   the maximum acceleration of the second DOF; must be
     *                     negative
     * @return the created acceleration segment
     */
    public static AccelerationSegment getDecelerationSegment(DualDOFPositionState decelerateTo, double dof1Vel,
            double dof2Vel, double dof1AccMax, double dof2AccMax) {
        double duration = Math.max(Math.abs(dof1Vel) / dof1AccMax, Math.abs(dof2Vel) / dof2AccMax);
        double dof1Acc = -dof1Vel / duration;
        double dof2Acc = -dof2Vel / duration;
        return new AccelerationSegment(
                new DualDOFState(decelerateTo.dof1Pos() + duration * duration * dof1Acc / 2,
                        decelerateTo.dof2Pos() + duration * duration * dof2Acc / 2, dof1Vel, dof2Vel),
                duration, dof1Acc, dof2Acc);
    }

    /**
     * Get an acceleration segment for a velocity change segment. A velocity change
     * segment is one where you come in with a certain velocity vector and come out
     * with a certain velocity vector, and you have a certain waypoint. The waypoint
     * is not where you pass through, rather it allows you to specify where you
     * would pass through if you slowed down to zero. Basically you take the two
     * tangents to the curve and find their intersection point and the waypoint
     * allows you to specify where that is.
     * 
     * @param waypoint       the waypoint
     * @param dof1InitialVel the incoming velocity of the first DOF
     * @param dof2InitialVel the incoming velocity of the second DOF
     * @param dof1FinalVel   the final velocity of the first DOF
     * @param dof2FinalVel   the final velocity of the second DOF
     * @param dof1AccMax     the maximum acceleration of the first DOF
     * @param dof2AccMax     the maximum acceleration of the second DOF
     * @return the constructed acceleration segment representing the velocity change
     *         segment
     */
    public static AccelerationSegment getVelocityChangeSegment(DualDOFPositionState waypoint, double dof1InitialVel,
            double dof2InitialVel, double dof1FinalVel, double dof2FinalVel, double dof1AccMax, double dof2AccMax) {
        double duration = Math.max(Math.abs(dof1FinalVel - dof1InitialVel) / dof1AccMax,
                Math.abs(dof2FinalVel - dof2InitialVel) / dof2AccMax);
        double dof1Acc = (dof1FinalVel - dof1InitialVel) / duration;
        double dof2Acc = (dof2FinalVel - dof2InitialVel) / duration;
        double dof1Travel = dof1InitialVel * duration + duration * duration * dof1Acc / 2;
        double dof2Travel = dof2InitialVel * duration + duration * duration * dof2Acc / 2;
        // We now need to solve the following system of linear equations:
        // (in order to determine which point our starting point is if the waypoint is the origin)
        // (x + dof1Travel) * dof2FinalVel - (y + dof2Travel) * dof1FinalVel = 0 (1)
        // x * dof2InitialVel - y * dof1InitialVel = 0 (2)
        // (1) can be simplified to:
        // x * dof2FinalVel - y * dof1FinalVel = dof2Travel * dof1FinalVel - dof1Travel * dof2FinalVel
        var solVector = MatBuilder
                .fill(N2.instance, N2.instance, dof2FinalVel, -dof1FinalVel, dof2InitialVel, -dof1InitialVel).inv()
                .times(VecBuilder.fill(dof2Travel * dof1FinalVel - dof1Travel * dof2FinalVel, 0));
        DualDOFState solInitialState = new DualDOFState(waypoint.dof1Pos() + solVector.get(0, 0),
                waypoint.dof2Pos() + solVector.get(1, 0), dof1InitialVel, dof2InitialVel);
        return new AccelerationSegment(solInitialState, duration, dof1Acc, dof2Acc);

    }

    public double getDuration() {
        return duration;
    }

    public DualDOFState getInitialState() {
        return initialState;
    }

    public DualDOFState getFinalState() {
        return calculate(duration);
    }

    public DualDOFState calculate(double time) {
        return new DualDOFState(initialState.dof1Pos() + time * initialState.dof1Vel() + time * time * dof1Acc / 2,
                initialState.dof2Pos() + time * initialState.dof2Vel() + time * time * dof2Acc / 2,
                initialState.dof1Vel() + time * dof1Acc, initialState.dof2Vel() + time * dof2Acc);
    }
}
