/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The LobstahPoseEstimator class, based on PhotonPoseEstimator, filters or
 * combines readings from all the AprilTags visible at a given timestamp on the
 * field to produce a single robot in field pose, using the strategy set below.
 * Example usage can be found in our apriltagExample example project.
 * 
 * Differences from PhotonPoseEstimator: - No multi tag on rio strategy - method
 * signatures are slightly different because no camera calibration data is
 * needed as there is no multi tag on rio strategy
 */
public class LobstahPoseEstimator {
    private static int InstanceCount = 0;

    private AprilTagFieldLayout fieldTags;
    private TargetModel tagModel = TargetModel.kAprilTag36h11;
    private PoseStrategy primaryStrategy;
    private PoseStrategy multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;
    private Transform3d robotToCamera;

    private Pose3d lastPose;
    private Pose3d referencePose;
    protected double poseCacheTimestampSeconds = -1;
    private final Set<Integer> reportedErrors = new HashSet<>();

    /**
     * Create a new LobstahPoseEstimator.
     *
     * @param fieldTags     A WPILib {@link AprilTagFieldLayout} linking AprilTag
     *                      IDs to Pose3d objects with respect to the FIRST field
     *                      using the <a href=
     *                      "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
     *                      Coordinate System</a>. Note that setting the origin of
     *                      this layout object will affect the results from this
     *                      class.
     * @param strategy      The strategy it should use to determine the best pose.
     * @param robotToCamera {@link Transform3d} from the center of the robot to the
     *                      camera mount position (ie, robot ➔ camera) in the
     *                      <a href=
     *                      "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     *                      Coordinate System</a>.
     */
    public LobstahPoseEstimator(AprilTagFieldLayout fieldTags, PoseStrategy strategy, Transform3d robotToCamera) {
        this.fieldTags = fieldTags;
        setPrimaryStrategy(strategy);
        this.robotToCamera = robotToCamera;

        HAL.report(tResourceType.kResourceType_PhotonPoseEstimator, InstanceCount);
        InstanceCount++;
    }

    /** Invalidates the pose cache. */
    private void invalidatePoseCache() {
        poseCacheTimestampSeconds = -1;
    }

    private void checkUpdate(Object oldObj, Object newObj) {
        if (oldObj != newObj && oldObj != null && !oldObj.equals(newObj)) { invalidatePoseCache(); }
    }

    /**
     * Get the AprilTagFieldLayout being used by the PositionEstimator.
     *
     * <p>
     * Note: Setting the origin of this layout will affect the results from this
     * class.
     *
     * @return the AprilTagFieldLayout
     */
    public AprilTagFieldLayout getFieldTags() {
        return fieldTags;
    }

    /**
     * Set the AprilTagFieldLayout being used by the PositionEstimator.
     *
     * <p>
     * Note: Setting the origin of this layout will affect the results from this
     * class.
     *
     * @param fieldTags the AprilTagFieldLayout
     */
    public void setFieldTags(AprilTagFieldLayout fieldTags) {
        checkUpdate(this.fieldTags, fieldTags);
        this.fieldTags = fieldTags;
    }

    /**
     * Get the TargetModel representing the tags being detected. This is used for
     * on-rio multitag.
     *
     * <p>
     * By default, this is {@link TargetModel#kAprilTag36h11}.
     */
    public TargetModel getTagModel() {
        return tagModel;
    }

    /**
     * Set the TargetModel representing the tags being detected. This is used for
     * on-rio multitag.
     *
     * @param tagModel E.g. {@link TargetModel#kAprilTag36h11}.
     */
    public void setTagModel(TargetModel tagModel) {
        this.tagModel = tagModel;
    }

    /**
     * Get the Position Estimation Strategy being used by the Position Estimator.
     *
     * @return the strategy
     */
    public PoseStrategy getPrimaryStrategy() {
        return primaryStrategy;
    }

    /**
     * Set the Position Estimation Strategy used by the Position Estimator.
     *
     * @param strategy the strategy to set
     */
    public void setPrimaryStrategy(PoseStrategy strategy) {
        if (strategy == PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
            DriverStation.reportError("LobstahPoseEstimator doesn't support on-rio multi-tag! Keeping at "
                    + this.primaryStrategy.toString(), false);
            return;
        }
        checkUpdate(this.primaryStrategy, strategy);
        this.primaryStrategy = strategy;
    }

    /**
     * Set the Position Estimation Strategy used in multi-tag mode when only one tag
     * can be seen. Must NOT be MULTI_TAG_PNP
     *
     * @param strategy the strategy to set
     */
    public void setMultiTagFallbackStrategy(PoseStrategy strategy) {
        checkUpdate(this.multiTagFallbackStrategy, strategy);
        if (strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR || strategy == PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
            DriverStation.reportWarning("Fallback cannot be set to MULTI_TAG_PNP! Setting to lowest ambiguity", false);
            strategy = PoseStrategy.LOWEST_AMBIGUITY;
        }
        this.multiTagFallbackStrategy = strategy;
    }

    /**
     * Return the reference position that is being used by the estimator.
     *
     * @return the referencePose
     */
    public Pose3d getReferencePose() {
        return referencePose;
    }

    /**
     * Update the stored reference pose for use when using the
     * <b>CLOSEST_TO_REFERENCE_POSE</b> strategy.
     *
     * @param referencePose the referencePose to set
     */
    public void setReferencePose(Pose3d referencePose) {
        checkUpdate(this.referencePose, referencePose);
        this.referencePose = referencePose;
    }

    /**
     * Update the stored reference pose for use when using the
     * <b>CLOSEST_TO_REFERENCE_POSE</b> strategy.
     *
     * @param referencePose the referencePose to set
     */
    public void setReferencePose(Pose2d referencePose) {
        setReferencePose(new Pose3d(referencePose));
    }

    /**
     * Update the stored last pose. Useful for setting the initial estimate when
     * using the <b>CLOSEST_TO_LAST_POSE</b> strategy.
     *
     * @param lastPose the lastPose to set
     */
    public void setLastPose(Pose3d lastPose) {
        this.lastPose = lastPose;
    }

    /**
     * Update the stored last pose. Useful for setting the initial estimate when
     * using the <b>CLOSEST_TO_LAST_POSE</b> strategy.
     *
     * @param lastPose the lastPose to set
     */
    public void setLastPose(Pose2d lastPose) {
        setLastPose(new Pose3d(lastPose));
    }

    /**
     * @return The current transform from the center of the robot to the camera
     *         mount position
     */
    public Transform3d getRobotToCameraTransform() {
        return robotToCamera;
    }

    /**
     * Useful for pan and tilt mechanisms and such.
     *
     * @param robotToCamera The current transform from the center of the robot to
     *                      the camera mount position
     */
    public void setRobotToCameraTransform(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    /**
     * Updates the estimated position of the robot. Returns empty if:
     *
     * <ul>
     * <li>The timestamp of the provided pipeline result is the same as in the
     * previous call to {@code update()}.
     * <li>No targets were found in the pipeline results.
     * </ul>
     *
     * @return an {@link LobstahEstimatedRobotPose} with an estimated pose,
     *         timestamp, and targets used to create the estimate.
     */
    public Optional<LobstahEstimatedRobotPose> update(PhotonPipelineResult cameraResult) {
        // Time in the past -- give up, since the following if expects times > 0
        if (cameraResult.getTimestampSeconds() < 0) { return Optional.empty(); }

        // If the pose cache timestamp was set, and the result is from the same
        // timestamp, return an
        // empty result
        if (poseCacheTimestampSeconds > 0
                && Math.abs(poseCacheTimestampSeconds - cameraResult.getTimestampSeconds()) < 1e-6) {
            return Optional.empty();
        }

        // Remember the timestamp of the current result used
        poseCacheTimestampSeconds = cameraResult.getTimestampSeconds();

        // If no targets seen, trivial case -- return empty result
        if (!cameraResult.hasTargets()) { return Optional.empty(); }

        return update(cameraResult, this.primaryStrategy);
    }

    private Optional<LobstahEstimatedRobotPose> update(PhotonPipelineResult cameraResult, PoseStrategy strat) {
        Optional<LobstahEstimatedRobotPose> estimatedPose = Optional.empty();
        switch (strat) {
            case LOWEST_AMBIGUITY:
                estimatedPose = lowestAmbiguityStrategy(cameraResult);
                break;
            case CLOSEST_TO_REFERENCE_POSE:
                estimatedPose = closestToReferencePoseStrategy(cameraResult, referencePose);
                break;
            case CLOSEST_TO_LAST_POSE:
                setReferencePose(lastPose);
                estimatedPose = closestToReferencePoseStrategy(cameraResult, referencePose);
                break;
            case AVERAGE_BEST_TARGETS:
                estimatedPose = averageBestTargetsStrategy(cameraResult);
                break;
            case MULTI_TAG_PNP_ON_COPROCESSOR:
                estimatedPose = multiTagOnCoprocStrategy(cameraResult);
                break;
            default:
                DriverStation.reportError("[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
                return Optional.empty();
        }

        return estimatedPose;
    }

    private Optional<LobstahEstimatedRobotPose> multiTagOnCoprocStrategy(PhotonPipelineResult result) {
        if (result.getMultiTagResult().isPresent()) {
            var estimatedPose = result.getMultiTagResult().get().estimatedPose;
            var best_tf = estimatedPose.best;
            var best = new Pose3d().plus(best_tf) // field-to-camera
                    .relativeTo(fieldTags.getOrigin()).plus(robotToCamera.inverse()); // field-to-robot
            var alt_tf = estimatedPose.alt;
            var alt = new Pose3d().plus(alt_tf) // field-to-camera
                    .relativeTo(fieldTags.getOrigin()).plus(robotToCamera.inverse()); // field-to-robot
            return Optional.of(new LobstahEstimatedRobotPose(best, alt, estimatedPose.bestReprojErr,
                    estimatedPose.altReprojErr, result.getTimestampSeconds(), estimatedPose.ambiguity,
                    result.getTargets(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
        } else {
            return update(result, this.multiTagFallbackStrategy);
        }
    }

    /**
     * Return the estimated position of the robot with the lowest position ambiguity
     * from a List of pipeline results.
     *
     * @param result pipeline result
     * @return the estimated position of the robot in the FCS and the estimated
     *         timestamp of this estimation.
     */
    private Optional<LobstahEstimatedRobotPose> lowestAmbiguityStrategy(PhotonPipelineResult result) {
        PhotonTrackedTarget lowestAmbiguityTarget = null;

        double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : result.targets) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            // Make sure the target is a Fiducial target.
            if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = targetPoseAmbiguity;
                lowestAmbiguityTarget = target;
            }
        }

        // Although there are confirmed to be targets, none of them may be fiducial
        // targets.
        if (lowestAmbiguityTarget == null) return Optional.empty();

        int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

        Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

        if (targetPosition.isEmpty()) {
            reportFiducialPoseError(targetFiducialId);
            return Optional.empty();
        }

        return Optional.of(new LobstahEstimatedRobotPose(
                targetPosition.get().transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                        .transformBy(robotToCamera.inverse()),
                new Pose3d(), 1, 1, result.getTimestampSeconds(), lowestAmbiguityTarget.getPoseAmbiguity(),
                result.getTargets(), PoseStrategy.LOWEST_AMBIGUITY));
    }

    /**
     * Return the estimated position of the robot using the target with the lowest
     * delta in the vector magnitude between it and the reference pose.
     *
     * @param result        pipeline result
     * @param referencePose reference pose to check vector magnitude difference
     *                      against.
     * @return the estimated position of the robot in the FCS and the estimated
     *         timestamp of this estimation.
     */
    private Optional<LobstahEstimatedRobotPose> closestToReferencePoseStrategy(PhotonPipelineResult result,
            Pose3d referencePose) {
        if (referencePose == null) {
            DriverStation.reportError(
                    "[PhotonPoseEstimator] Tried to use reference pose strategy without setting the reference!", false);
            return Optional.empty();
        }

        double smallestPoseDelta = 10e9;
        LobstahEstimatedRobotPose lowestDeltaPose = null;

        for (PhotonTrackedTarget target : result.targets) {
            int targetFiducialId = target.getFiducialId();

            // Don't report errors for non-fiducial targets. This could also be resolved by
            // adding -1 to
            // the initial HashSet.
            if (targetFiducialId == -1) continue;

            Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

            if (targetPosition.isEmpty()) {
                reportFiducialPoseError(targetFiducialId);
                continue;
            }

            Pose3d altTransformPosition = targetPosition.get()
                    .transformBy(target.getAlternateCameraToTarget().inverse()).transformBy(robotToCamera.inverse());
            Pose3d bestTransformPosition = targetPosition.get().transformBy(target.getBestCameraToTarget().inverse())
                    .transformBy(robotToCamera.inverse());

            double altDifference = Math.abs(calculateDifference(referencePose, altTransformPosition));
            double bestDifference = Math.abs(calculateDifference(referencePose, bestTransformPosition));

            if (altDifference < smallestPoseDelta) {
                smallestPoseDelta = altDifference;
                lowestDeltaPose = new LobstahEstimatedRobotPose(altTransformPosition, bestTransformPosition, 1, 0,
                        result.getTimestampSeconds(), 1, result.getTargets(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            }
            if (bestDifference < smallestPoseDelta) {
                smallestPoseDelta = bestDifference;
                lowestDeltaPose = new LobstahEstimatedRobotPose(bestTransformPosition, altTransformPosition, 1, 0,
                        result.getTimestampSeconds(), 1, result.getTargets(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            }
        }
        return Optional.ofNullable(lowestDeltaPose);
    }

    /**
     * Return the average of the best target poses using ambiguity as weight.
     *
     * @param result pipeline result
     * @return the estimated position of the robot in the FCS and the estimated
     *         timestamp of this estimation.
     */
    private Optional<LobstahEstimatedRobotPose> averageBestTargetsStrategy(PhotonPipelineResult result) {
        List<Pair<PhotonTrackedTarget, Pose3d>> estimatedRobotPoses = new ArrayList<>();
        double totalAmbiguity = 0;

        for (PhotonTrackedTarget target : result.targets) {
            int targetFiducialId = target.getFiducialId();

            // Don't report errors for non-fiducial targets. This could also be resolved by
            // adding -1 to
            // the initial HashSet.
            if (targetFiducialId == -1) continue;

            Optional<Pose3d> targetPosition = fieldTags.getTagPose(target.getFiducialId());

            if (targetPosition.isEmpty()) {
                reportFiducialPoseError(targetFiducialId);
                continue;
            }

            double targetPoseAmbiguity = target.getPoseAmbiguity();

            // Pose ambiguity is 0, use that pose
            if (targetPoseAmbiguity == 0) {
                return Optional.of(new LobstahEstimatedRobotPose(
                        targetPosition.get().transformBy(target.getBestCameraToTarget().inverse())
                                .transformBy(robotToCamera.inverse()),
                        new Pose3d(), 1, 0, result.getTimestampSeconds(), 0, result.getTargets(),
                        PoseStrategy.AVERAGE_BEST_TARGETS));
            }

            totalAmbiguity += 1.0 / target.getPoseAmbiguity();

            estimatedRobotPoses.add(new Pair<>(target, targetPosition.get()
                    .transformBy(target.getBestCameraToTarget().inverse()).transformBy(robotToCamera.inverse())));
        }

        // Take the average

        Translation3d transform = new Translation3d();
        Rotation3d rotation = new Rotation3d();

        if (estimatedRobotPoses.isEmpty()) return Optional.empty();

        for (Pair<PhotonTrackedTarget, Pose3d> pair : estimatedRobotPoses) {
            // Total ambiguity is non-zero confirmed because if it was zero, that pose was
            // returned.
            double weight = (1.0 / pair.getFirst().getPoseAmbiguity()) / totalAmbiguity;
            Pose3d estimatedPose = pair.getSecond();
            transform = transform.plus(estimatedPose.getTranslation().times(weight));
            rotation = rotation.plus(estimatedPose.getRotation().times(weight));
        }

        return Optional.of(new LobstahEstimatedRobotPose(new Pose3d(transform, rotation), new Pose3d(), 1, 0,
                result.getTimestampSeconds(), totalAmbiguity, result.getTargets(), PoseStrategy.AVERAGE_BEST_TARGETS));
    }

    /**
     * Difference is defined as the vector magnitude between the two poses
     *
     * @return The absolute "difference" (>=0) between two Pose3ds.
     */
    private double calculateDifference(Pose3d x, Pose3d y) {
        return x.getTranslation().getDistance(y.getTranslation());
    }

    private void reportFiducialPoseError(int fiducialId) {
        if (!reportedErrors.contains(fiducialId)) {
            DriverStation.reportError("[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId,
                    false);
            reportedErrors.add(fiducialId);
        }
    }
}
