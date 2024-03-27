package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    Transform3d robotToCam = VisionConstants.CAMERA_OFFSET;

    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;

    public VisionSubsystem() {
        camera = new PhotonCamera(VisionConstants.TARGET_CAMERA);
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        camera,
                        robotToCam);

        Shuffleboard.getTab("vision").addNumber("Tag ID", this::getTargetId);
        Shuffleboard.getTab("vision").addNumber("Tag Yaw", () -> getYaw().orElse(0.0));
        Shuffleboard.getTab("vision")
                .addNumber("Distance From Target", () -> getGroundDistanceFromTarget());
    }

    /**
     * @return whether or not an AprilTag is detected
     */
    public boolean hasTarget() {
        var result = getResult();
        var hasTarget = result.hasTargets();
        if (!hasTarget) return false;

        for (var target : result.getTargets()) {
            int id = target.getFiducialId();
            if (id == VisionConstants.APRILTAG_SPEAKER_CENTER_BLUE
                    || id == VisionConstants.APRILTAG_SPEAKER_CENTER_RED) {
                return hasTarget;
            }
        }

        return false;
    }

    /**
     * (don't use this unless you're desperate)
     *
     * @return the PhotonCamera object
     */
    public PhotonCamera getCamera() {
        return camera;
    }

    /**
     * @return the best target's ID
     */
    public int getTargetId() {
        var results = getResult();
        if (!results.hasTargets()) return -1;
        return results.getBestTarget().getFiducialId();
    }

    public List<PhotonTrackedTarget> getTargets() {
        var result = getResult();
        if (result.hasTargets()) {
            return result.getTargets();
        } else {
            return new ArrayList<PhotonTrackedTarget>();
        }
    }

    /**
     * @return the distance from the target in meters
     */
    public double getGroundDistanceFromTarget() {
        var result = getResult();
        if (result.hasTargets()) {
            Transform3d transform = result.getBestTarget().getBestCameraToTarget();
            return Math.hypot(transform.getX(), transform.getY());
        } else {
            return 0;
        }
    }

    /**
     * @returns the vision pipeline's getResult() (all of its data)
     */
    public PhotonPipelineResult getResult() {
        return camera.getLatestResult();
    }

    /**
     * @return the yaw offset of the best target
     */
    public Optional<Double> getYaw() {
        var result = getResult();
        if (!result.hasTargets()) return Optional.empty();

        for (var target : result.getTargets()) {
            int id = target.getFiducialId();
            if (id == VisionConstants.APRILTAG_SPEAKER_CENTER_BLUE
                    || id == VisionConstants.APRILTAG_SPEAKER_CENTER_RED) {
                return Optional.of(target.getYaw());
            }
        }

        return Optional.empty();
    }

    public boolean isAligned() {
        // 10 is outside of target lock
        return Math.abs(getYaw().orElse(10.0)) <= VisionConstants.TARGET_LOCK_RANGE;
    }

    /** angle the arm should be at to shoot */
    public double getAngle() { // TODO: not been set up yet
        // https://desmos.com/calculator/u5civq4pfs
        return 90;
    }

    /**
     * WARNING: Currently causes a loop overrun, do not use
     *
     * @return a Pose3d representing the robot's position on the field
     */
    public Pose3d getRobotPose() {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        return pose.get().estimatedPose;
    }
}
