package frc.robot.profile;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DualDOFProfile {
    private final List<DualDOFProfileSegment> segments;
    private final List<Double> startTimes;
    private final double duration;

    private DualDOFProfile(List<DualDOFProfileSegment> segments) {
        this.segments = segments;
        startTimes = new ArrayList<>(segments.size());
        double startTime = 0;
        for (DualDOFProfileSegment segment : segments) {
            startTimes.add(startTime);
            startTime += segment.getDuration();
        }
        duration = startTime;
    }

    /**
     * Create a new dual-DOF profile from a list of waypoints and velocity and
     * acceleration constraints for the two DOFs. The constraints are treated as
     * limits on the absolute value of the velocity and acceleration; velocity and
     * acceleration will be negative if necessary but their magnitude will never
     * exceed the constraints.
     * 
     * @param waypoints       a list of waypoints
     * @param dof1Constraints constraints on the first DOF
     * @param dof2Constraints constraints on the second DOF
     * @return the constructed profile
     */
    public static DualDOFProfile fromWaypoints(List<DualDOFPositionState> waypoints,
            TrapezoidProfile.Constraints dof1Constraints, TrapezoidProfile.Constraints dof2Constraints) {
        if (waypoints.size() < 2)
            throw new IllegalArgumentException("waypoints must have at least 2 waypoints");
        DualDOFPositionState start = waypoints.get(0);
        DualDOFPositionState end = waypoints.get(waypoints.size() - 1);
        List<Pair<Double, Double>> velocities = new ArrayList<>(waypoints.size() - 1);
        for (int i = 0; i < waypoints.size() - 1; i++) {
            final DualDOFPositionState segmentStart = waypoints.get(i), segmentEnd = waypoints.get(i + 1);
            final double dof1Vel, dof2Vel;
            if (Math.abs(segmentEnd.dof1Pos() - segmentStart.dof1Pos())
                    / Math.abs(segmentEnd.dof2Pos() - segmentStart.dof2Pos()) > dof1Constraints.maxVelocity
                            / dof2Constraints.maxVelocity) {
                dof1Vel = Math.copySign(dof1Constraints.maxVelocity, segmentEnd.dof1Pos() - segmentStart.dof1Pos());
                dof2Vel = (segmentEnd.dof2Pos() - segmentStart.dof2Pos())
                        / (segmentEnd.dof1Pos() - segmentStart.dof1Pos()) * dof1Vel;
            } else {
                dof2Vel = Math.copySign(dof2Constraints.maxVelocity, segmentEnd.dof2Pos() - segmentStart.dof2Pos());
                dof1Vel = (segmentEnd.dof1Pos() - segmentStart.dof1Pos())
                        / (segmentEnd.dof2Pos() - segmentStart.dof2Pos()) * dof2Vel;
            }
            velocities.add(new Pair<>(dof1Vel, dof2Vel));
        }
        List<DualDOFProfileSegment> segments = new ArrayList<>(2 * waypoints.size() - 1);
        segments.add(AccelerationSegment.getAccelerationSegment(start, velocities.get(0).getFirst(),
                velocities.get(0).getSecond(), dof1Constraints.maxAcceleration, dof2Constraints.maxAcceleration));
        for (int i = 1; i < waypoints.size() - 1; i++) {
            var segment = AccelerationSegment.getVelocityChangeSegment(waypoints.get(i),
                    velocities.get(i - 1).getFirst(), velocities.get(i - 1).getSecond(), velocities.get(i).getFirst(),
                    velocities.get(i).getSecond(), dof1Constraints.maxAcceleration, dof2Constraints.maxAcceleration);
            if ((segment.getInitialState().dof1Pos() - segments.get(i - 1).getFinalState().dof1Pos())
                    / velocities.get(i - 1).getFirst() < 0
                    || (segment.getInitialState().dof2Pos() - segments.get(i - 1).getFinalState().dof2Pos())
                            / velocities.get(i - 1).getSecond() < 0) {
                velocities.set(i - 1,
                        new Pair<>(0.95 * velocities.get(i - 1).getFirst(), 0.95 * velocities.get(i - 1).getSecond()));
                segments.remove(i - 1);
                if (i == 1) {
                    i--;
                    segments.add(AccelerationSegment.getAccelerationSegment(start, velocities.get(0).getFirst(),
                            velocities.get(0).getSecond(), dof1Constraints.maxAcceleration,
                            dof2Constraints.maxAcceleration));
                    continue;
                }
                i -= 2;
                continue;
            }
            segments.add(segment);
        }
        var segment = AccelerationSegment.getDecelerationSegment(end, velocities.get(velocities.size() - 1).getFirst(),
                velocities.get(velocities.size() - 1).getSecond(), dof1Constraints.maxAcceleration,
                dof2Constraints.maxAcceleration);
        int j = waypoints.size() - 1;
        while ((segment.getInitialState().dof1Pos() - segments.get(j - 1).getFinalState().dof1Pos())
                / velocities.get(j - 1).getFirst() < 0
                || (segment.getInitialState().dof2Pos() - segments.get(j - 1).getFinalState().dof2Pos())
                        / velocities.get(j - 1).getSecond() < 0) {
            velocities.set(j - 1,
                    new Pair<>(0.95 * velocities.get(j - 1).getFirst(), 0.95 * velocities.get(j - 1).getSecond()));
            segments.remove(j - 1);
            if (j == 1) {
                segments.add(AccelerationSegment.getAccelerationSegment(start, velocities.get(0).getFirst(),
                        velocities.get(0).getSecond(), dof1Constraints.maxAcceleration,
                        dof2Constraints.maxAcceleration));
            } else {
                segments.add(AccelerationSegment.getVelocityChangeSegment(waypoints.get(j - 1),
                        velocities.get(j - 2).getFirst(), velocities.get(j - 2).getSecond(),
                        velocities.get(j - 1).getFirst(),
                        velocities.get(j - 1).getSecond(), dof1Constraints.maxAcceleration,
                        dof2Constraints.maxAcceleration));
            }
            segment = AccelerationSegment.getDecelerationSegment(end, velocities.get(velocities.size() - 1).getFirst(),
                    velocities.get(velocities.size() - 1).getSecond(), dof1Constraints.maxAcceleration,
                    dof2Constraints.maxAcceleration);
        }
        segments.add(segment);
        for (int i = 0; i < waypoints.size() - 1; i++) {
            segments.add(2 * i + 1, new SteadyStateSegment(segments.get(2 * i).getFinalState(),
                    segments.get(2 * i + 1).getInitialState()));
        }
        return new DualDOFProfile(segments);
    }

    /**
     * Get the duration of this profile
     * 
     * @return the duration in seconds
     */
    public double getDuration() {
        return duration;
    }

    /**
     * Calculate the state at the desired time after the start of the profile
     * 
     * @param time time after the start of the profile in seconds
     * @return the calculated state
     */
    public DualDOFState calculate(double time) {
        if (time > duration)
            return segments.get(segments.size() - 1).getFinalState();
        int i = 0;
        if (time >= startTimes.get(startTimes.size() - 1))
            i = startTimes.size() - 1;
        else
            while (time >= startTimes.get(i + 1))
                i++;
        double startTime = startTimes.get(i);
        return segments.get(i).calculate(time - startTime);
    }
}
