package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private static PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    private Transform3d robotToCam =
            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));

    private PhotonPoseEstimator photonPoseEstimator =
            new PhotonPoseEstimator(
                    VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                    camera,
                    robotToCam);

    public PhotonPipelineResult getResult() {
        return camera.getLatestResult();
    }

    public boolean hasTarget() {
        return getResult().hasTargets();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return getResult().getTargets();
    }

    public double getPoseFromTarget() {
        PhotonTrackedTarget target = getTargets().get(0);
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        VisionConstants.CAMERA_HEIGHT,
                        VisionConstants.APRILTAG_RED_SHOOTER_HEIGHT,
                        VisionConstants.CAMERA_PITCH,
                        Units.degreesToRadians(target.getPitch()));

        return range;
    }

    double getRangeMeters() {
        double range = PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);
        return range;
    }

    public Pose3d getRobotPose() {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        return pose.get().estimatedPose;
    }
}
