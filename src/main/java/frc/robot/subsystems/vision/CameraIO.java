package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public Pose3d bestEstimatedPose = Pose3d.kZero;
        public Pose3d altEstimatedPose = Pose3d.kZero;

        public double bestReprojErr = 0.0;
        public double altReprojErr = 0.0;

        public double ambiguity = 0.0;

        public double estimatedPoseTimestamp = 0.0;

        public int[] visibleFiducialIDs = new int[] {};

        public double totalArea = 0.0;

        public void updateFrom(LobstahEstimatedRobotPose estimatedRobotPose) {
            bestEstimatedPose = estimatedRobotPose.bestEstimatedPose;
            altEstimatedPose = estimatedRobotPose.alternateEstimatedPose;
            bestReprojErr = estimatedRobotPose.bestReprojError;
            altReprojErr = estimatedRobotPose.altReprojError;
            ambiguity = estimatedRobotPose.multiTagAmbiguity;
            estimatedPoseTimestamp = estimatedRobotPose.timestampSeconds;
            visibleFiducialIDs = estimatedRobotPose.fiducialIDsUsed;
            totalArea = estimatedRobotPose.totalArea;
        }

        public void clearInputs() {
            bestEstimatedPose = null;
            altEstimatedPose = null;
            bestReprojErr = 0;
            altReprojErr = 0;
            ambiguity = 0;
            visibleFiducialIDs = new int[] {};
            totalArea = 0.0;
        }
    }

    public List<PhotonTrackedTarget> getTrackedTargets();

    public void updateInputs(CameraIOInputs inputs, Pose3d robotPose);

    public String getCameraName();

    public Pose3d[] getTagPoses();
}
