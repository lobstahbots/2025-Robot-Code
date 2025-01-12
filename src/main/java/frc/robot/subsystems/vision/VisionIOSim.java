package frc.robot.subsystems.vision;

/*
 * MIT License
 *
 * Copyright (c) PhotonVision & FRC 246
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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class VisionIOSim implements VisionIO {
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2024Crescendo);
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;
    private final SimCameraProperties frontCameraProp = new SimCameraProperties();
    private final SimCameraProperties rearCameraProp = new SimCameraProperties();
    private final PhotonCameraSim frontCameraSim;
    private final PhotonCameraSim rearCameraSim;
    private final LobstahPoseEstimator frontPoseEstimator;
    private final LobstahPoseEstimator rearPoseEstimator;
    private LobstahEstimatedRobotPose estimatedFrontPose = new LobstahEstimatedRobotPose(new Pose3d(), new Pose3d(), 0, 0, 0, 0,
            new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private LobstahEstimatedRobotPose estimatedRearPose = new LobstahEstimatedRobotPose(new Pose3d(), new Pose3d(), 0, 0, 0, 0,
            new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private final Map<String, PhotonCameraSim> camSimMap = new HashMap<>();
    private static final double kBufferLengthSeconds = 1.5;
    // save robot-to-camera for each camera over time (Pose3d is easily
    // interpolatable)
    private final Map<PhotonCameraSim, TimeInterpolatableBuffer<Pose3d>> camTrfMap = new HashMap<>();
    // interpolate drivetrain with twists
    private final TimeInterpolatableBuffer<Pose3d> robotPoseBuffer = TimeInterpolatableBuffer
            .createBuffer(kBufferLengthSeconds);
    private final Map<String, Set<VisionTargetSim>> targetSets = new HashMap<>();
    private final Field2d dbgField;
    private final Transform3d kEmptyTrf = new Transform3d();

    /**
     * Create a new simulated {@link VisionIO}. The rear camera is named
     * {@code photonvision_rear} and the front camera is named
     * {@code photonvision_front}.
     */
    public VisionIOSim() {
        addAprilTags(aprilTagFieldLayout);
        this.rearCamera = new PhotonCamera("photonvision_rear");
        this.frontCamera = new PhotonCamera("photonvision_front");
        this.frontCameraProp.setCalibration(VisionConstants.CAMERA_RES_WIDTH, VisionConstants.CAMERA_RES_HEIGHT,
                Rotation2d.fromDegrees(VisionConstants.CAMERA_FOV_DEG));
        this.frontCameraProp.setCalibError(VisionConstants.AVG_ERROR_PX, VisionConstants.ERROR_STDEV_PX);
        this.frontCameraProp.setFPS(VisionConstants.FPS);
        this.frontCameraProp.setAvgLatencyMs(VisionConstants.CAMERA_AVG_LATENCY_MS);
        this.frontCameraProp.setLatencyStdDevMs(VisionConstants.CAMERA_LATENCY_STDEV_MS);
        this.rearCameraProp.setCalibration(VisionConstants.CAMERA_RES_WIDTH, VisionConstants.CAMERA_RES_HEIGHT,
                Rotation2d.fromDegrees(VisionConstants.CAMERA_FOV_DEG));
        this.rearCameraProp.setCalibError(VisionConstants.AVG_ERROR_PX, VisionConstants.ERROR_STDEV_PX);
        this.rearCameraProp.setFPS(VisionConstants.FPS);
        this.rearCameraProp.setAvgLatencyMs(VisionConstants.CAMERA_AVG_LATENCY_MS);
        this.rearCameraProp.setLatencyStdDevMs(VisionConstants.CAMERA_LATENCY_STDEV_MS);
        this.frontCameraSim = new PhotonCameraSim(frontCamera, frontCameraProp);
        this.rearCameraSim = new PhotonCameraSim(rearCamera, rearCameraProp);
        addCamera(frontCameraSim, VisionConstants.ROBOT_TO_FRONT_CAMERA);
        addCamera(rearCameraSim, VisionConstants.ROBOT_TO_REAR_CAMERA);
        this.frontPoseEstimator = new LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY,
                VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.rearPoseEstimator = new LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY,
                VisionConstants.ROBOT_TO_REAR_CAMERA);
        dbgField = new Field2d();
        String tableName = "VisionSystemSim-main";
        SmartDashboard.putData(tableName + "/Sim Field", dbgField);
    }

    public void addCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
        var existing = camSimMap.putIfAbsent(cameraSim.getCamera().getName(), cameraSim);
        if (existing == null) {
            camTrfMap.put(cameraSim, TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds));
            camTrfMap.get(cameraSim).addSample(Timer.getFPGATimestamp(), new Pose3d().plus(robotToCamera));
        }
    }

    /**
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of
     * the camera relative to the subset of these targets which are visible from the
     * given camera position.
     *
     * <p>
     * The AprilTags from this layout will be added as vision targets under the type
     * "apriltag". The poses added preserve the tag layout's current alliance
     * origin. If the tag layout's alliance origin is changed, these added tags will
     * have to be cleared and re-added.
     *
     * @param tagLayout The field tag layout to get Apriltag poses and IDs from
     */
    public void addAprilTags(AprilTagFieldLayout tagLayout) {
        for (AprilTag tag : tagLayout.getTags()) {
            addVisionTargets("apriltag", new VisionTargetSim(tagLayout.getTagPose(tag.ID).get(), // preserve alliance rotation
                    TargetModel.kAprilTag36h11, tag.ID));
        }
    }

    /**
     * Adds targets on the field which your vision system is designed to detect. The
     * {@link PhotonCamera}s simulated from this system will report the location of
     * the camera relative to the subset of these targets which are visible from the
     * given camera position.
     *
     * @param type    Type of target (e.g. "cargo").
     * @param targets Targets to add to the simulated field
     */
    public void addVisionTargets(String type, VisionTargetSim... targets) {
        targetSets.computeIfAbsent(type, k -> new HashSet<>());
        for (var tgt : targets) {
            targetSets.get(type).add(tgt);
        }
    }

    /*
     * Updates the vision sim with the robot pose.
     * 
     * @param robotPose The current pose of the robot.
     */
    public void updateInputs(VisionIOInputs inputs, Pose3d robotPoseMeters) {
        var targetTypes = targetSets.entrySet();
        // update vision targets on field
        targetTypes.forEach(entry -> dbgField.getObject(entry.getKey())
                .setPoses(entry.getValue().stream().map(t -> t.getPose().toPose2d()).collect(Collectors.toList())));

        if (robotPoseMeters == null) return;

        // save "real" robot poses over time
        double now = Timer.getFPGATimestamp();
        robotPoseBuffer.addSample(now, robotPoseMeters);
        dbgField.setRobotPose(robotPoseMeters.toPose2d());

        var allTargets = new ArrayList<VisionTargetSim>();
        targetTypes.forEach((entry) -> allTargets.addAll(entry.getValue()));
        var visTgtPoses2d = new ArrayList<Pose2d>();
        var cameraPoses2d = new ArrayList<Pose2d>();
        boolean processed = false;
        // process each camera
        for (var camSim : camSimMap.values()) {
            // check if this camera is ready to process and get latency
            var optTimestamp = camSim.consumeNextEntryTime();
            if (optTimestamp.isEmpty())
                continue;
            else
                processed = true;
            // when this result "was" read by NT
            long timestampNT = optTimestamp.get();
            // this result's processing latency in milliseconds
            double latencyMillis = camSim.prop.estLatencyMs();
            // the image capture timestamp in microseconds of this result
            long timestampCapture = timestampNT - (long) (latencyMillis * 1e3);

            // use camera pose from the image capture timestamp
            Pose3d lateRobotPose = getRobotPose(timestampCapture);
            Pose3d lateCameraPose = lateRobotPose.plus(getRobotToCamera(camSim, timestampCapture).get());
            cameraPoses2d.add(lateCameraPose.toPose2d());

            // process a PhotonPipelineResult with visible targets
            var camResult = camSim.process(latencyMillis, lateCameraPose, allTargets);
            camResult = new PhotonPipelineResult(camResult.metadata.sequenceID, timestampCapture,
                    camResult.metadata.publishTimestampMicros, camResult.metadata.timeSinceLastPong,
                    camResult.getTargets(), camResult.getMultiTagResult());
            // pipelineResults.put(camSim.getCamera().getName(), camResult);

            // publish this info to NT at estimated timestamp of receive
            camSim.submitProcessedFrame(camResult, timestampNT);
            // display debug results

            for (var target : camResult.getTargets()) {
                var trf = target.getBestCameraToTarget();
                if (trf.equals(kEmptyTrf)) continue;
                visTgtPoses2d.add(lateCameraPose.transformBy(trf).toPose2d());
            }

            if (camSim.getCamera().getName().equals(frontCamera.getName())) {
                Optional<LobstahEstimatedRobotPose> frontPoseOptional = frontPoseEstimator.update(camResult);
                if (frontPoseOptional.isPresent()) {
                    estimatedFrontPose = frontPoseOptional.get();
                    inputs.bestEstimatedFrontPose = estimatedFrontPose.bestEstimatedPose;
                    inputs.altEstimatedFrontPose = estimatedFrontPose.alternateEstimatedPose;
                    inputs.bestFrontReprojErr = estimatedFrontPose.bestReprojError;
                    inputs.altFrontReprojErr = estimatedFrontPose.altReprojError;
                    inputs.estimatedFrontPoseTimestamp = estimatedFrontPose.timestampSeconds;
                    inputs.visibleFrontFiducialIDs = estimatedFrontPose.targetsUsed.stream()
                            .mapToInt((target) -> target.getFiducialId()).toArray();
                    inputs.frontTotalArea = estimatedFrontPose.targetsUsed.stream()
                            .mapToDouble((target) -> target.getArea() / 100).sum();
                    inputs.frontAmbiguity = estimatedFrontPose.multiTagAmbiguity;
                }
            } else {
                Optional<LobstahEstimatedRobotPose> rearPoseOptional = rearPoseEstimator.update(camResult);
                if (rearPoseOptional.isPresent()) {
                    estimatedRearPose = rearPoseOptional.get();
                    inputs.bestEstimatedRearPose = estimatedRearPose.bestEstimatedPose;
                    inputs.altEstimatedRearPose = estimatedRearPose.alternateEstimatedPose;
                    inputs.bestRearReprojErr = estimatedRearPose.bestReprojError;
                    inputs.altRearReprojErr = estimatedRearPose.altReprojError;
                    inputs.estimatedRearPoseTimestamp = estimatedRearPose.timestampSeconds;
                    inputs.visibleRearFiducialIDs = estimatedRearPose.targetsUsed.stream()
                            .mapToInt((target) -> target.getFiducialId()).toArray();
                    inputs.rearTotalArea = estimatedRearPose.targetsUsed.stream()
                            .mapToDouble((target) -> target.getArea() / 100).sum();
                    inputs.rearAmbiguity = estimatedRearPose.multiTagAmbiguity;
                }
            }

        }
        if (processed) dbgField.getObject("visibleTargetPoses").setPoses(visTgtPoses2d);
        if (!cameraPoses2d.isEmpty()) dbgField.getObject("cameras").setPoses(cameraPoses2d);
    }

    public List<PhotonTrackedTarget> getFrontTrackedTargets() {
        return estimatedFrontPose.targetsUsed;
    }

    public List<PhotonTrackedTarget> getRearTrackedTargets() {
        return estimatedRearPose.targetsUsed;
    }

    /**
     * Get the robot pose in meters saved by the vision system at this timestamp.
     *
     * @param timestamp Timestamp of the desired robot pose
     */
    public Pose3d getRobotPose(double timestamp) {
        return robotPoseBuffer.getSample(timestamp).orElse(new Pose3d());
    }

    /**
     * Get a simulated camera's position relative to the robot. If the requested
     * camera is invalid, an empty optional is returned.
     *
     * @param cameraSim   The specific camera to get the robot-to-camera transform
     *                    of
     * @param timeSeconds Timestamp in seconds of when the transform should be
     *                    observed
     * @return The transform of this camera, or an empty optional if it is invalid
     */
    public Optional<Transform3d> getRobotToCamera(PhotonCameraSim cameraSim, double timeSeconds) {
        var trfBuffer = camTrfMap.get(cameraSim);
        if (trfBuffer == null) return Optional.empty();
        var sample = trfBuffer.getSample(timeSeconds);
        if (sample.isEmpty()) return Optional.empty();
        return Optional.of(new Transform3d(new Pose3d(), sample.orElse(new Pose3d())));
    }

    @AutoLogOutput
    public Pose3d[] getFrontTagPoses() {
        var frontTargets = estimatedFrontPose.targetsUsed;
        Pose3d[] frontTagPoses = new Pose3d[frontTargets.size()];
        for (int i = 0; i < frontTagPoses.length; i++) {
            frontTagPoses[i] = aprilTagFieldLayout.getTagPose(frontTargets.get(i).getFiducialId()).get();
        }
        return frontTagPoses;
    }

    @AutoLogOutput
    public Pose3d[] getRearTagPoses() {
        var rearTargets = estimatedRearPose.targetsUsed;
        Pose3d[] rearTagPoses = new Pose3d[rearTargets.size()];
        for (int i = 0; i < rearTagPoses.length; i++) {
            rearTagPoses[i] = aprilTagFieldLayout.getTagPose(rearTargets.get(i).getFiducialId()).get();
        }
        return rearTagPoses;
    }
}