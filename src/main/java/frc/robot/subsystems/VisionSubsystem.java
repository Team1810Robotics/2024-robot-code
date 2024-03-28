package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    public VisionSubsystem() {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        Shuffleboard.getTab("vision").addNumber("Tag ID", this::getTargetId);
        Shuffleboard.getTab("vision").addNumber("Tag Yaw", () -> getSpeakerYaw().orElse(0.0));
        Shuffleboard.getTab("vision").addNumber("Distance", this::getDistanceFromSpeakerTarget);
        Shuffleboard.getTab("vision").addNumber("angle", this::getAngle);
    }

    /**
     * @return whether or not an AprilTag is detected
     */
    public boolean hasSpeakerTarget() {
        var result = getResult();
        var hasTarget = result.hasTargets();
        if (!hasTarget) return false;

        for (var target : result.getTargets()) {
            int id = target.getFiducialId();
            if (isSpeakerTarget(id)) {
                return hasTarget;
            }
        }

        return false;
    }

    /**
     * @return whether or not an AprilTag is detected
     */
    public boolean hasSpeakerTarget(PhotonPipelineResult result) {
        var hasTarget = result.hasTargets();
        if (!hasTarget) return false;

        for (var target : result.getTargets()) {
            int id = target.getFiducialId();
            if (isSpeakerTarget(id)) return hasTarget;
        }

        return false;
    }

    /**
     * @return whether or not an AprilTag is detected
     */
    public boolean hasTarget() {
        return getResult().hasTargets();
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
    public double getDistanceFromTarget() {
        var result = getResult();
        if (result.hasTargets()) {
            Transform3d transform = result.getBestTarget().getBestCameraToTarget();
            return Math.hypot(transform.getX(), transform.getY());
        } else {
            return 0;
        }
    }

    /**
     * @return the distance from the target in meters
     */
    public double getDistanceFromSpeakerTarget() {
        return speakerTargets(
                (PhotonTrackedTarget target) -> {
                    Transform3d transform = target.getBestCameraToTarget();
                    return Math.hypot(transform.getX(), transform.getY());
                },
                0.0);
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
    public Optional<Double> getSpeakerYaw() {
        return speakerTargets(
                (PhotonTrackedTarget target) -> Optional.of(target.getYaw()), Optional.empty());
    }

    /**
     * @return the yaw offset of the best target
     */
    public Optional<Double> getYaw() {
        var result = getResult();
        if (!result.hasTargets()) return Optional.empty();

        return Optional.of(result.getBestTarget().getYaw());
    }

    public boolean isAligned() {
        // VisionConstants.TARGET_LOCK_RANGE + 1 is outside the range
        // meaning if it's outside the range, it's not aligned
        double yaw = getSpeakerYaw().orElse(VisionConstants.TARGET_LOCK_RANGE + 1);
        return Math.abs(yaw) <= VisionConstants.TARGET_LOCK_RANGE;
    }

    /** angle the arm should be at to shoot */
    // https://desmos.com/calculator/u5civq4pfs
    public double getAngle() {
        double distance = getDistanceFromSpeakerTarget();
        if (distance == 0.0) return 62.0;

        double c = -4.91756 * Math.pow(distance, 2);
        double b = 35.6183 * distance;
        double a = 19.4615;

        return a + b + c;
    }

    private boolean isSpeakerTarget(int id) {
        return id == VisionConstants.APRILTAG_SPEAKER_CENTER_BLUE
                || id == VisionConstants.APRILTAG_SPEAKER_CENTER_RED;
    }

    private <T extends Object> T speakerTargets(Function<PhotonTrackedTarget, T> fn, T other) {
        var result = getResult();
        if (!result.hasTargets()) return other;

        for (var target : result.getTargets()) {
            if (isSpeakerTarget(target.getFiducialId())) return fn.apply(target);
        }

        return other;
    }
}
