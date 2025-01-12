package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2024Crescendo);
    private final LobstahPoseEstimator frontPoseEstimator;
    private final LobstahPoseEstimator rearPoseEstimator;
    private LobstahEstimatedRobotPose estimatedFrontPose = new LobstahEstimatedRobotPose(null, null, 0.0, 0.0, 0.0, 0.0,
            new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private LobstahEstimatedRobotPose estimatedRearPose = new LobstahEstimatedRobotPose(null, null, 0.0, 0.0, 0.0, 0.0,
            new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private final Alert frontDisconnectedAlert;
    private final Alert rearDisconnectedAlert;

    /**
     * Create a new {@link VisionIO} instance using PhotonVision cameras & pose
     * estimation. Rear camera is named {@code photonvision1} and front camera
     * {@code photonvision2}.
     */
    public VisionIOPhoton() {
        this.rearCamera = new PhotonCamera("photonvision1");
        this.frontCamera = new PhotonCamera("photonvision2");
        this.frontPoseEstimator = new LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY,
                VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.rearPoseEstimator = new LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY,
                VisionConstants.ROBOT_TO_REAR_CAMERA);
        frontDisconnectedAlert = new Alert("Front camera has disconnected.", AlertType.kError);
        rearDisconnectedAlert = new Alert("Rear camera has disconnected.", AlertType.kError);
        frontDisconnectedAlert.set(false);
        rearDisconnectedAlert.set(false);
    }

    public void updateInputs(VisionIOInputs inputs, Pose3d robotPoseMeters) {
        List<PhotonPipelineResult> frontPoseResults = frontCamera.getAllUnreadResults();
        if (frontPoseResults.size() > 0) {
            Optional<LobstahEstimatedRobotPose> frontPoseOptional = frontPoseEstimator
                    .update(frontPoseResults.get(frontPoseResults.size() - 1));
            if (frontPoseOptional.isPresent()) {
                estimatedFrontPose = frontPoseOptional.get();

                inputs.bestEstimatedFrontPose = estimatedFrontPose.bestEstimatedPose;
                inputs.altEstimatedFrontPose = estimatedFrontPose.alternateEstimatedPose;
                inputs.bestFrontReprojErr = estimatedFrontPose.bestReprojError;
                inputs.altFrontReprojErr = estimatedFrontPose.altReprojError;
                inputs.estimatedFrontPoseTimestamp = estimatedFrontPose.timestampSeconds;
                inputs.visibleFrontFiducialIDs = estimatedFrontPose.fiducialIDsUsed;
                inputs.frontTotalArea = estimatedFrontPose.totalArea;
                inputs.frontAmbiguity = estimatedFrontPose.multiTagAmbiguity;
            } else {
                inputs.bestEstimatedFrontPose = null;
                inputs.altEstimatedFrontPose = null;
                inputs.bestFrontReprojErr = 0.0;
                inputs.visibleFrontFiducialIDs = new int[0];
                inputs.frontTotalArea = 0;
                inputs.frontAmbiguity = 0;
                inputs.altFrontReprojErr = 0;
            }
        }
        frontDisconnectedAlert.set(!frontCamera.isConnected());

        List<PhotonPipelineResult> rearPoseResults = rearCamera.getAllUnreadResults();
        if (rearPoseResults.size() > 0) {
            Optional<LobstahEstimatedRobotPose> rearPoseOptional = rearPoseEstimator
                    .update(frontPoseResults.get(frontPoseResults.size() - 1));
            if (rearPoseOptional.isPresent()) {
                estimatedRearPose = rearPoseOptional.get();

                inputs.bestEstimatedRearPose = estimatedRearPose.bestEstimatedPose;
                inputs.altEstimatedRearPose = estimatedRearPose.alternateEstimatedPose;
                inputs.bestRearReprojErr = estimatedRearPose.bestReprojError;
                inputs.altRearReprojErr = estimatedRearPose.altReprojError;
                inputs.estimatedRearPoseTimestamp = estimatedRearPose.timestampSeconds;
                inputs.visibleRearFiducialIDs = estimatedRearPose.fiducialIDsUsed;
                inputs.rearTotalArea = estimatedRearPose.totalArea;
                inputs.rearAmbiguity = estimatedRearPose.multiTagAmbiguity;
            } else {
                inputs.bestEstimatedRearPose = null;
                inputs.altEstimatedRearPose = null;
                inputs.bestRearReprojErr = 0.0;
                inputs.visibleRearFiducialIDs = new int[0];
                inputs.rearTotalArea = 0;
                inputs.rearAmbiguity = 0;
                inputs.altRearReprojErr = 0;
            }
        }
        rearDisconnectedAlert.set(!rearCamera.isConnected());
    }

    public List<PhotonTrackedTarget> getFrontTrackedTargets() {
        return estimatedFrontPose.targetsUsed;
    }

    public List<PhotonTrackedTarget> getRearTrackedTargets() {
        return estimatedRearPose.targetsUsed;
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
