package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        /**
         * Estimated robot pose with least reprojection error
         */
        public Pose3d bestEstimatedPose = Pose3d.kZero;
        /**
         * Estimated robot pose with most reprojection error
         */
        public Pose3d altEstimatedPose = Pose3d.kZero;
        /**
         * Reprojection error of best robot pose
         */
        public double bestReprojErr = 0.0;
        /**
         * Reprojection error of worst robot pose
         */
        public double altReprojErr = 0.0;
        /**
         * Multi-tag ambiguity - is equal to {@code bestReprojError} divided by
         * {@code altReprojError}
         */
        public double ambiguity = 0.0;
        /**
         * Estimated time of the pose
         */
        public double estimatedPoseTimestamp = 0.0;
        /**
         * Visible fiducial IDs
         */
        public int[] visibleFiducialIDs = new int[] {};
        /**
         * Total area of targets as a fraction of the total image area
         */
        public double totalArea = 0.0;

        /**
         * The photon pipeline result
         */
        public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
        
        public boolean connected = false;

        /**
         * Update from a {@link LobstahEstimatedRobotPose}.
         * 
         * @param estimatedRobotPose pose to get information from
         */
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

        /**
         * Set all inputs to their empty value.
         */
        public void clearInputs() {
            bestEstimatedPose = Pose3d.kZero;
            altEstimatedPose = Pose3d.kZero;
            bestReprojErr = 0;
            altReprojErr = 0;
            ambiguity = 0;
            visibleFiducialIDs = new int[] {};
            totalArea = 0.0;
            pipelineResult = new PhotonPipelineResult();
        }
    }

    /**
     * Get a list of the tracked targets.
     * 
     * @return the list of tracked targets
     */
    public List<PhotonTrackedTarget> getTrackedTargets();

    public void updateInputs(CameraIOInputs inputs);

    /**
     * Get the name of this camera
     * 
     * @return the name as a string
     */
    public String getCameraName();

    public Pose3d[] getTagPoses();
}
