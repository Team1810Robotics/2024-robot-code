package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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

    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    Transform3d robotToCam = VisionConstants.CAMERA_OFFSET;

    // int[] goodTargets = VisionConstants.GOOD_TARGETS;

    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;

    int speakerTargetID = 0;

    public VisionSubsystem() {
        camera = new PhotonCamera(VisionConstants.TARGET_CAMERA);
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        camera,
                        robotToCam);
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
        var target = getResult().hasTargets();
        if (!target) return -1;
        return getResult().getBestTarget().getFiducialId();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return getResult().getTargets();
    }

    /**
     * @return the distance from the target in meters
     */
    public double getPoseFromTarget() {
        PhotonTrackedTarget target = getTargets().get(0);
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        robotToCam.getZ(),
                        target.getBestCameraToTarget().getZ(),
                        robotToCam.getRotation().getZ(),
                        Units.degreesToRadians(target.getPitch()));

        return range;
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
        if (hasTarget()) {
            return Optional.of(getResult().getBestTarget().getYaw());
        } else {
            return Optional.empty();
        }
    }

    public boolean isAligned() {
        var yaw = getYaw();
        if (yaw.isEmpty()) return false;

        return Math.abs(getYaw().get()) <= VisionConstants.TARGET_LOCK_RANGE;
    }

    /**
     * @return the yaw offset for the speaker AprilTags
     */
    public Optional<Double> getSpeakerYaw() {
        Optional<Double> speakerYaw = Optional.empty();
        speakerTargetID = 0;
        if (hasTarget()) {
            for (int i = 0; i < getResult().getTargets().size(); i++) {
                if (getResult().getTargets().get(i).getFiducialId() == 4) {
                    speakerYaw = Optional.of(getResult().getTargets().get(i).getYaw());
                    speakerTargetID = 4;
                    System.out.println("Sending Red Speaker Yaw " + speakerYaw);
                }
                if (getResult().getTargets().get(i).getFiducialId() == 7) {
                    speakerYaw = Optional.of(getResult().getTargets().get(i).getYaw());
                    speakerTargetID = 7;
                    System.out.println("Sending Blue Speaker Yaw " + speakerYaw);
                }
                if (getResult().getTargets().get(i).getFiducialId() == 6) {
                    speakerYaw = Optional.of(getResult().getTargets().get(i).getYaw());
                    speakerTargetID = 6; //sam is hot
                    System.out.println("Sending Blue Amp Yaw " + speakerYaw);
                }
                if (getResult().getTargets().get(i).getFiducialId() == 5) {
                    speakerYaw = Optional.of(getResult().getTargets().get(i).getYaw());
                    speakerTargetID = 5;
                    System.out.println("Sending Red Amp Yaw " + speakerYaw);
                }
            }
            if ((getResult().getTargets().size() == 1)
                    && ((speakerTargetID != 4) && (speakerTargetID != 7) && (speakerTargetID != 5) && (speakerTargetID != 6))) {
                // System.out.println("Falling back to best target");
                speakerYaw = Optional.of(getResult().getBestTarget().getYaw());
            }
        } //I love ian weatherman
        if (hasTarget() == false) {
            speakerYaw = Optional.empty();
            // System.out.println("No Target");
        }
        return speakerYaw;
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
