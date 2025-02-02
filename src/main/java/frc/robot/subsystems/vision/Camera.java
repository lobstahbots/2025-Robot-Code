package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.math.LobstahMath;

public class Camera {
    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
    private static Pose2d robotPose = new Pose2d();
    public final String cameraName;

    public Camera(CameraIO io) {
        this.io = io;
        this.cameraName = io.getCameraName();
    }

    public Pose getEstimatedPose(Pose2d odometryPose) {
        robotPose = odometryPose;
        if (inputs.visibleFiducialIDs.length == 0) return Pose.empty();
        Pose3d resolvedPose = null;
        double resolvedReprojErr = 0;

        double bestDistanceToCurrPose = LobstahMath.getDistBetweenPoses(inputs.bestEstimatedPose, odometryPose);
        double altDistanceToCurrPose = LobstahMath.getDistBetweenPoses(inputs.altEstimatedPose, odometryPose);
        double bestRotationDiff = Math.abs(
                inputs.bestEstimatedPose.toPose2d().getRotation().minus(odometryPose.getRotation()).getRotations());
        double altRotationDiff = Math
                .abs(inputs.altEstimatedPose.toPose2d().getRotation().minus(odometryPose.getRotation()).getRotations());

        // Select pose if closest to robot pose or better rotation, reject if reprojection error > 0.4 * alternative pose reprojection error
        if ((bestDistanceToCurrPose <= altDistanceToCurrPose || bestRotationDiff <= altRotationDiff)
                && inputs.bestReprojErr <= VisionConstants.REPROJECTION_ERROR_REJECTION_THRESHOLD
                        * inputs.altReprojErr) {
            resolvedPose = inputs.bestEstimatedPose;
            resolvedReprojErr = inputs.bestReprojErr;
        }
        // Otherwise, select alt pose if ambiguity is low enough and alt solution is closest to robot pose and better rotation
        // harder to use more ambiguous pose so has to be both closer and better rotation
        // Reprojection error of alternate pose will always be higher than that of alternate pose
        else if (inputs.ambiguity <= VisionConstants.AMBIGUITY_ACCEPTANCE_THRESHOLD
                && altDistanceToCurrPose <= bestDistanceToCurrPose && bestRotationDiff <= altRotationDiff) {
            resolvedPose = inputs.altEstimatedPose;
            resolvedReprojErr = inputs.altReprojErr;
        }

        Vector<N3> stdev = VisionConstants.BASE_STDEV
                .times(Math.pow(resolvedReprojErr, VisionConstants.REPROJ_TO_STDEV_EXP) // Start with reprojection error
                        * Math.exp(1 / inputs.visibleFiducialIDs.length)
                        * Math.pow(inputs.visibleFiducialIDs.length, VisionConstants.APRIL_TAG_NUMBER_EXPONENT) // Multiply by the scaling for the number of AprilTags
                        * Math.pow(inputs.totalArea, 1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE) * Math.log(2)
                        / Math.log(inputs.totalArea + 1) // Multiply by the scaling for the area of the AprilTags
                );

        Logger.recordOutput("Vision/" + cameraName + "/ResolvedPose", resolvedPose);
        Logger.recordOutput("Vision/" + cameraName + "/stdev", stdev.toString());

        if (resolvedPose != null && resolvedPose.getX() == 0 && resolvedPose.getY() == 0) resolvedPose = null;

        return new Pose(Optional.ofNullable(resolvedPose), Optional.ofNullable(stdev));
    }

    /**
     * Get the timestamp of the pose capture.
     * 
     * @return the latest timestamp.
     */
    public double getTimestamp() {
        return inputs.estimatedPoseTimestamp;
    }

    /**
     * Get the tracked targets from the camera.
     * 
     * @return A list of the {@link PhotonTrackedTarget}s.
     */
    public List<PhotonTrackedTarget> getTargets() {
        return io.getTrackedTargets();
    }

    /**
     * Get the fiducial IDs of the targets seen by the camera.
     * 
     * @return an array of the IDs
     */
    public int[] getFiducialIDs() {
        return inputs.visibleFiducialIDs;
    }

    public void periodic() {
        io.updateInputs(inputs, new Pose3d(robotPose));
        Logger.processInputs("Vision/" + cameraName, inputs);
    }

    /**
     * Contains a pose and a stdev estimated from a camera.
     */
    public static record Pose(
            /**
             * The estimated robot pose.
             */
            Optional<Pose3d> pose, /**
                                    * The stdev for the estimated robot pose.
                                    */
            Optional<Vector<N3>> stdev) {
        public static Pose empty() {
            return new Pose(Optional.empty(), Optional.empty());
        }
    };
}
