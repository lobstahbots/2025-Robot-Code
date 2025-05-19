package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public class Camera {
    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
    public final String cameraName;

    public Camera(CameraIO io) {
        this.io = io;
        this.cameraName = io.getCameraName();
    }

    /**
     * Get the estimated pose from this camera.
     * 
     * @param odometryPose The current odometry pose
     * @return a {@link Pose} object representing the estimated pose, standard
     *         deviation, and timestamp
     */
    public Pose getEstimatedPose(Pose2d odometryPose) {
        if (inputs.visibleFiducialIDs.length == 0) return Pose.empty();
        Pose3d resolvedPose = null;
        double resolvedReprojErr = 0;

        if (inputs.ambiguity < VisionConstants.AMBIGUITY_ACCEPTANCE_THRESHOLD) {
            resolvedPose = inputs.bestEstimatedPose;
            resolvedReprojErr = inputs.bestReprojErr;
        }

        Vector<N3> stdev = VisionConstants.BASE_STDEV
                .times(Math.pow(resolvedReprojErr, VisionConstants.REPROJ_TO_STDEV_EXP) // Start with reprojection error
                        * 10 * inputs.ambiguity // multiply by ambiguity
                        * Math.exp(1 / inputs.visibleFiducialIDs.length)
                        * Math.pow(inputs.visibleFiducialIDs.length, VisionConstants.APRIL_TAG_NUMBER_EXPONENT) // Multiply by the scaling for the number of AprilTags
                        * Math.pow(inputs.totalArea, -1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE) * Math.log(2)
                        / Math.log(inputs.totalArea + Math.E) // Multiply by the scaling for the area of the AprilTags
                );

        Logger.recordOutput("Vision/" + cameraName + "/ResolvedPose", resolvedPose);
        Logger.recordOutput("Vision/" + cameraName + "/stdev", stdev.toString());

        if (resolvedPose != null && resolvedPose.getX() == 0 && resolvedPose.getY() == 0) resolvedPose = null;

        return new Pose(Optional.ofNullable(resolvedPose), Optional.ofNullable(stdev),
                Optional.of(inputs.estimatedPoseTimestamp));
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
        io.updateInputs(inputs);
        Logger.processInputs("Vision/" + cameraName, inputs);
    }

    /**
     * Get this camera's name
     * 
     * @return The camera name, as set by the {@link CameraIO} passed into the
     *         constructor of this lassF
     */
    public String getName() {
        return cameraName;
    }

    /**
     * Get the robot-to-camera transform for this camera.
     * 
     * @return The robot-to-camera transform for this camera, as a
     *         {@link Transform3d}.
     */
    public Transform3d getRobotToCamera() {
        return VisionConstants.CAMERA_TRANSFORMS.get(cameraName);
    }

    /**
     * Contains a pose, a timestamp, and a stdev estimated from a camera.
     */
    public static record Pose(
            /**
             * The estimated robot pose.
             */
            Optional<Pose3d> pose,
            /**
             * The stdev for the estimated robot pose.
             */
            Optional<Vector<N3>> stdev, /**
                                         * The timestamp of the estimated pose
                                         */
            Optional<Double> timestamp) {
        public static Pose empty() {
            return new Pose(Optional.empty(), Optional.empty(), Optional.empty());
        }
    };
}
