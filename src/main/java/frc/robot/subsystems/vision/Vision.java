package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private Pose2d robotPose = new Pose2d();

    public Vision(VisionIO io) {
        this.io = io;
    }

    /**
     * Get the estimated pose from both cameras.
     * 
     * @param odometryPose The current pose returned by the robot odometry to filter
     *                     the vision estimate in comparison to.
     * @return The estimated pose, as {@link Poses}, containing both front & rear
     *         poses and their respective standard deviation vectors.
     */
    public Poses getEstimatedPose(Pose2d odometryPose) {
        Pose3d resolvedFrontPose = null;
        double resolvedFrontReprojErr = 0.0;
        Pose3d resolvedRearPose = null;
        double resolvedRearReprojErr = 0.0;
        Vector<N3> frontStdev = null;
        Vector<N3> rearStdev = null;

        if (inputs.visibleFrontFiducialIDs.length > 0) {
            // Select pose if ambiguity is low enough or if closest to robot pose, reject if reprojection error > 0.4 * alternative pose reprojection error
            if (Math.abs(inputs.bestEstimatedFrontPose.toPose2d().getRotation().minus(odometryPose.getRotation())
                    .getRadians()) <= Math
                            .abs(inputs.altEstimatedFrontPose.toPose2d().getRotation().minus(odometryPose.getRotation())
                                    .getRadians())) {
                resolvedFrontPose = inputs.bestEstimatedFrontPose;
                resolvedFrontReprojErr = inputs.bestFrontReprojErr;
            }
            // Otherwise, select alt pose if ambiguity is high enough and alt solution is closest to robot pose, reject if reprojection error > 0.4 * best pose reprojection error
            else if (Math.abs(inputs.altEstimatedFrontPose.toPose2d().getRotation().minus(odometryPose.getRotation())
                    .getRadians()) <= Math
                            .abs(inputs.bestEstimatedFrontPose.toPose2d().getRotation()
                                    .minus(odometryPose.getRotation()).getRadians())) {
                resolvedFrontPose = inputs.altEstimatedFrontPose;
                resolvedFrontReprojErr = inputs.altFrontReprojErr;
            }
            frontStdev = VisionConstants.BASE_STDEV
                    .times(Math.pow(resolvedFrontReprojErr, VisionConstants.AMBIGUITY_TO_STDEV_EXP) // Start with reprojection error
                            * Math.exp(1 / inputs.visibleFrontFiducialIDs.length)
                            * Math.pow(inputs.visibleFrontFiducialIDs.length, VisionConstants.APRIL_TAG_NUMBER_EXPONENT) // Multiply by the scaling for the number of AprilTags
                            * Math.pow(inputs.frontTotalArea, 1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE)
                            * Math.log(2) / Math.log(inputs.frontTotalArea + 1) // Multiply by the scaling for the area of the AprilTags
                    );

            Logger.recordOutput("Best Front Pose", inputs.bestEstimatedFrontPose);
            Logger.recordOutput("Alt Front Pose", inputs.altEstimatedFrontPose);
            Logger.recordOutput("Resolved front", resolvedFrontPose.toPose2d());
        }

        if (inputs.visibleRearFiducialIDs.length > 0) {
            // Select pose if ambiguity is low enough or if closest to robot pose, reject if reprojection error > 0.4 * alternative pose reprojection error
            if (Math.abs(inputs.bestEstimatedRearPose.toPose2d().getRotation().minus(odometryPose.getRotation())
                    .getRadians()) <= Math
                            .abs(inputs.altEstimatedRearPose.toPose2d().getRotation().minus(odometryPose.getRotation())
                                    .getRadians())) {
                resolvedRearPose = inputs.bestEstimatedRearPose;
                resolvedRearReprojErr = inputs.bestRearReprojErr;
                Logger.recordOutput("Seen Rear", true);
            }
            // Otherwise, select alt pose if ambiguity is high enough and alt solution is closest to robot pose, reject if reprojection error > 0.4 * best pose reprojection error
            else if (Math.abs(inputs.altEstimatedRearPose.toPose2d().getRotation().minus(odometryPose.getRotation())
                    .getRadians()) <= Math
                            .abs(inputs.bestEstimatedRearPose.toPose2d().getRotation().minus(odometryPose.getRotation())
                                    .getRadians())) {
                resolvedRearPose = inputs.altEstimatedRearPose;
                resolvedRearReprojErr = inputs.altRearReprojErr;
                Logger.recordOutput("Seen Rear", true);
            }
            rearStdev = VisionConstants.BASE_STDEV
                    .times(Math.pow(resolvedRearReprojErr, VisionConstants.AMBIGUITY_TO_STDEV_EXP) // Start with reprojection error
                            * Math.exp(1 / inputs.visibleRearFiducialIDs.length)
                            * Math.pow(inputs.visibleRearFiducialIDs.length, VisionConstants.APRIL_TAG_NUMBER_EXPONENT) // Multiply by the scaling for the number of AprilTags
                            * Math.pow(inputs.rearTotalArea, 1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE)
                            * Math.log(2) / Math.log(inputs.rearTotalArea + 1) // Multiply by the scaling for the area of the AprilTags
                    );
            Logger.recordOutput("Best Rear Pose", inputs.bestEstimatedRearPose);
            Logger.recordOutput("Alt Rear Pose", inputs.altEstimatedRearPose);
            Logger.recordOutput("Resolved Rear", resolvedRearPose);
        }

        if (resolvedFrontPose != null && resolvedFrontPose.getX() == 0 && resolvedFrontPose.getY() == 0)
            resolvedFrontPose = null;
        if (resolvedRearPose != null && resolvedRearPose.getX() == 0 && resolvedRearPose.getY() == 0)
            resolvedRearPose = null;

        Poses poses = new Poses(Optional.ofNullable(resolvedFrontPose), Optional.ofNullable(resolvedRearPose),
                Optional.ofNullable(frontStdev), Optional.ofNullable(rearStdev));

        Logger.recordOutput("ront pose there?", poses.frontPose.isPresent());

        return poses;
    }

    /**
     * Get the timestamp of the front pose capture.
     * 
     * @return The latest timestamp.
     */
    public double getFrontTimestamp() {
        return inputs.estimatedFrontPoseTimestamp;
    }

    /**
     * Get the timestamp of the rear pose capture.
     * 
     * @return The latest timestamp.
     */
    public double getRearTimestamp() {
        return inputs.estimatedRearPoseTimestamp;
    }

    /**
     * Get the tracked targets from the front camera.
     * 
     * @return The tracked targets.
     */

    public List<PhotonTrackedTarget> getFrontTargets() {
        return io.getFrontTrackedTargets();
    }

    /**
     * Get the tracked targets from the rear camera.
     * 
     * @return The tracked targets.
     */
    public List<PhotonTrackedTarget> getRearTargets() {
        return io.getRearTrackedTargets();
    }

    /**
     * Get the fiducial IDs of the targets in the front camera.
     * 
     * @return A list of the IDs.
     */
    public int[] getFrontFiducialIDs() {
        return inputs.visibleFrontFiducialIDs;
    }

    /**
     * Get the fiducial IDs of the targets in the rear camera.
     * 
     * @return A list of the IDs.
     */
    public int[] getRearFiducialIDs() {
        return inputs.visibleRearFiducialIDs;
    }

    public void periodic() {
        io.updateInputs(inputs, new Pose3d(robotPose));
        Logger.processInputs("PhotonVision", inputs);
        io.periodic();
    }

    /**
     * Contains poses estimated from a front and rear camera. Both poses are
     * optional. In addition to the estimated poses, also contains standard
     * deviation vectors for both poses.
     */
    public record Poses(Optional<Pose3d> frontPose, Optional<Pose3d> rearPose, Optional<Vector<N3>> frontStdev,
            Optional<Vector<N3>> rearStdev) {};

    public void update(Pose2d robotPose) {
        this.robotPose = robotPose;
    }
}
