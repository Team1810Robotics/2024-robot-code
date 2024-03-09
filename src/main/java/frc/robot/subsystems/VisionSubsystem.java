package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
    private Transform3d robotToCam = VisionConstants.CAMERA_TO_ROBOT;

    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult result = new PhotonPipelineResult();
    private final DriveSubsystem drive;

    private final Field2d field = new Field2d();

    private double previousPipelineTimestamp;

    public VisionSubsystem(DriveSubsystem drivebase) {
        this.drive = drivebase;
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                        camera,
                        robotToCam);
    }

    /**
     * @return whether or not an AprilTag is detected
     */
    public boolean hasTarget() {
        return result.hasTargets();
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
        return result.getBestTarget().getFiducialId();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return result.getTargets();
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
     * @returns the vision pipeline's result (all of its data)
     */
    public PhotonPipelineResult getResult() {
        return result;
    }

    /**
     * @return the yaw offset of the best target
     */
    public Optional<Double> getYaw() {
        if (hasTarget()) {
            return Optional.of(result.getBestTarget().getYaw());
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
     * WARNING: Currently causes a loop overrun, do not use
     *
     * @return a Pose3d representing the robot's position on the field
     */
    public Pose3d getRobotPose() {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        return pose.get().estimatedPose;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();

        if (resultTimestamp == previousPipelineTimestamp || !pipelineResult.hasTargets()) return;
        previousPipelineTimestamp = resultTimestamp;
        PhotonTrackedTarget target = pipelineResult.getBestTarget();
        int fiducialId = target.getFiducialId();

        if (target.getPoseAmbiguity() >= VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD) return;
        var targetPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(fiducialId);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.get().transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(VisionConstants.CAMERA_TO_ROBOT);
        drive.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);

        field.setRobotPose(drive.getPose());
    }
}
