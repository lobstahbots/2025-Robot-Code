package frc.robot.subsystems.vision;

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
import frc.robot.Constants.VisionConstants;

public class CameraIOSim implements CameraIO {
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private final PhotonCamera camera;
    private final SimCameraProperties cameraProp = new SimCameraProperties();
    private final PhotonCameraSim cameraSim;
    private final LobstahPoseEstimator poseEstimator;
    private LobstahEstimatedRobotPose estimatedPose = new LobstahEstimatedRobotPose(null, null, 0, 0, 0, 0,
            new ArrayList<>(), VisionConstants.POSE_STRATEGY);
    private static final TimeInterpolatableBuffer<Pose3d> robotPoseBuffer = TimeInterpolatableBuffer
            .createBuffer(VisionConstants.SIM_BUFFER_LENGTH);
    private static final Map<String, Set<VisionTargetSim>> targetSets = new HashMap<>();
    private static final Field2d dbgField = new Field2d();
    private final String cameraName;
    private final Transform3d cameraToRobot;

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
    public static void addAprilTags(AprilTagFieldLayout tagLayout) {
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
    public static void addVisionTargets(String type, VisionTargetSim... targets) {
        targetSets.computeIfAbsent(type, k -> new HashSet<>());
        for (var tgt : targets) {
            targetSets.get(type).add(tgt);
        }
    }

    static {
        addAprilTags(aprilTagFieldLayout);
    }

    /**
     * Create a new simulated {@link CameraIO}.
     * 
     * @param cameraName The name of the camera, as published to NetworkTables.
     */
    public CameraIOSim(String cameraName) {
        this.cameraName = cameraName;
        cameraToRobot = VisionConstants.CAMERA_TRANSFORMS.get(cameraName);
        camera = new PhotonCamera(cameraName);
        cameraProp.setCalibration(VisionConstants.CAMERA_RES_WIDTH, VisionConstants.CAMERA_RES_HEIGHT,
                Rotation2d.fromDegrees(VisionConstants.CAMERA_FOV_DEG));
        cameraProp.setCalibError(VisionConstants.AVG_ERROR_PX, VisionConstants.ERROR_STDEV_PX);
        cameraProp.setFPS(VisionConstants.FPS);
        cameraProp.setAvgLatencyMs(VisionConstants.CAMERA_AVG_LATENCY_MS);
        cameraProp.setLatencyStdDevMs(VisionConstants.CAMERA_LATENCY_STDEV_MS);
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        poseEstimator = new LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, cameraToRobot);
    }

    public String getCameraName() {
        return cameraName;
    }

    public void updateInputs(CameraIOInputs inputs) {
        var allTargets = targetSets.entrySet().stream().flatMap(entry -> entry.getValue().stream()).toList();

        // check if this camera is ready to process and get latency
        var optTimestamp = cameraSim.consumeNextEntryTime();
        if (optTimestamp.isEmpty()) return;

        // when this result "was" read by NT
        long timestampNT = optTimestamp.get();
        // this results processing latency in milliseconds
        double latencyMillis = cameraSim.prop.estLatencyMs();
        // the image capture timestamp
        long timestampCapture = timestampNT - (long) (latencyMillis * 1e3);

        // use camera pose from the image capture timestamp
        Pose3d lateRobotPose = getRobotPose(timestampCapture);
        Pose3d lateCameraPose = lateRobotPose.plus(cameraToRobot);
        dbgField.getObject(cameraName).setPose(lateCameraPose.toPose2d());

        // process a PhotonPipelineResult with visible targets
        PhotonPipelineResult camResult = cameraSim.process(latencyMillis, lateCameraPose, allTargets);
        camResult = new PhotonPipelineResult(camResult.metadata.sequenceID, timestampCapture,
                camResult.metadata.publishTimestampMicros, camResult.metadata.timeSinceLastPong, camResult.getTargets(),
                camResult.getMultiTagResult());

        // publish this info to NT as estimated timestamp of receive
        cameraSim.submitProcessedFrame(camResult, timestampNT);

        // display debug results
        List<Pose2d> visibleTargetPoses = new ArrayList<>();
        for (PhotonTrackedTarget target : camResult.getTargets()) {
            Transform3d transform = target.getBestCameraToTarget();
            if (transform.getX() == 0 && transform.getY() == 0 && transform.getZ() == 0) continue;
            visibleTargetPoses.add(lateCameraPose.transformBy(transform).toPose2d());
        }
        dbgField.getObject("visibleTargetPoses" + cameraName).setPoses(visibleTargetPoses);

        Optional<LobstahEstimatedRobotPose> poseOptional = poseEstimator.update(camResult);
        if (poseOptional.isPresent()) {
            estimatedPose = poseOptional.get();
            inputs.updateFrom(estimatedPose);
            inputs.pipelineResult = camResult;
        } else
            inputs.clearInputs();
    }

    public List<PhotonTrackedTarget> getTrackedTargets() {
        return estimatedPose.targetsUsed;
    }

    @AutoLogOutput(key = "Vision/{cameraName}/tagPoses")
    public Pose3d[] getTagPoses() {
        return getTrackedTargets().stream()
                .map((PhotonTrackedTarget target) -> aprilTagFieldLayout.getTagPose(target.getFiducialId()).get())
                .toArray(Pose3d[]::new);
    }

    /**
     * Get the robot pose in meters saved by the vision system at this timestamp.
     *
     * @param timestamp Timestamp of the desired robot pose
     */
    public static Pose3d getRobotPose(double timestamp) {
        return robotPoseBuffer.getSample(timestamp).orElse(new Pose3d());
    }

    public static void periodic() {
        targetSets.entrySet().forEach(entry -> dbgField.getObject(entry.getKey())
                .setPoses(entry.getValue().stream().map(t -> t.getPose().toPose2d()).collect(Collectors.toList())));
    }

    public static void addSimPose(Pose3d robotPose) {
        robotPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPose);
    }
}
