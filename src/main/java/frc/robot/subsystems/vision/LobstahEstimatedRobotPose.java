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

import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An estimated pose based on pipeline result */
public class LobstahEstimatedRobotPose extends EstimatedRobotPose {
    /** The best estimated pose */
    public final Pose3d bestEstimatedPose;

    /** The alternate estimated pose */
    public final Pose3d alternateEstimatedPose;

    /** Ambiguity of the best pose */
    public final double bestReprojError;

    /** Ambiguity of the alternate pose */
    public final double altReprojError;

    public final int[] fiducialIDsUsed;

    public final double totalArea;

    public final double multiTagAmbiguity;

    /**
     * Constructs a LobstahEstimatedRobotPose
     * @param bestEstimatedPose The best estimated pose
     * @param alternateEstimatedPose The alternate estimated pose
     * @param bestReprojErr The reprojection error for the best estimated pose
     * @param altReprojErr The reprojection error for the alternate estimated pose
     * @param timestampSeconds timestamp of estimate
     * @param ambiguity the ambiguity
     * @param targetsUsed list of {@link PhotonTrackedTarget}s used
     * @param strategy the {@link PoseStrategy} used
     */
    public LobstahEstimatedRobotPose(Pose3d bestEstimatedPose, Pose3d alternateEstimatedPose, double bestReprojErr,
            double altReprojErr, double timestampSeconds, double ambiguity, List<PhotonTrackedTarget> targetsUsed,
            PoseStrategy strategy) {
        super(bestEstimatedPose, timestampSeconds, targetsUsed, strategy);
        var targetsSeen = targetsUsed.size();
        var visibleFiducialIDs = new int[targetsSeen];

        double area = 0;

        for (int i = 0; i < targetsSeen; i++) {
            var target = targetsUsed.get(i);
            visibleFiducialIDs[i] = target.getFiducialId();
            area += target.getArea() / 100; // Area is returned in percent but we want fraction
            // See
            // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#getting-data-from-a-target
        }
        this.bestEstimatedPose = bestEstimatedPose;
        this.alternateEstimatedPose = alternateEstimatedPose;
        this.bestReprojError = bestReprojErr;
        this.altReprojError = altReprojErr;
        this.fiducialIDsUsed = visibleFiducialIDs;
        this.totalArea = area;
        this.multiTagAmbiguity = ambiguity;
    }
}
