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

public class CameraIOPhoton implements CameraIO {
    private final PhotonCamera camera;
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private final LobstahPoseEstimator poseEstimator;
    private LobstahEstimatedRobotPose estimatedPose = new LobstahEstimatedRobotPose(null, null, 0, 0, 0, 0,
            new ArrayList<>(), VisionConstants.POSE_STRATEGY);
    private final Alert disconnectedAlert;
    private final String cameraName;

    public CameraIOPhoton(String cameraName) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        poseEstimator = new LobstahPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY,
                VisionConstants.CAMERA_TRANSFORMS.get(cameraName));
        disconnectedAlert = new Alert(cameraName + " has disconnected.", AlertType.kError);
        disconnectedAlert.set(false);
    }

    public void updateInputs(CameraIOInputs inputs) {
        List<PhotonPipelineResult> poseResults = camera.getAllUnreadResults();
        if (poseResults.size() > 0) {
            PhotonPipelineResult latestResult = poseResults.get(poseResults.size() - 1);
            Optional<LobstahEstimatedRobotPose> poseOptional = poseEstimator
                    .update(latestResult);
            if (poseOptional.isPresent()) {
                estimatedPose = poseOptional.get();
                inputs.updateFrom(estimatedPose);
                inputs.pipelineResult = latestResult;
            } else {
                inputs.clearInputs();
            }
        }
        inputs.connected = camera.isConnected();
        disconnectedAlert.set(!camera.isConnected());
    }

    public String getCameraName() {
        return cameraName;
    }

    public List<PhotonTrackedTarget> getTrackedTargets() {
        return estimatedPose.targetsUsed;
    }

    @AutoLogOutput(key = "Vision/{cameraName}/TagPoses")
    public Pose3d[] getTagPoses() {
        return getTrackedTargets().stream()
                .map((PhotonTrackedTarget target) -> aprilTagFieldLayout.getTagPose(target.getFiducialId()).get())
                .toArray(Pose3d[]::new);
    }
}
