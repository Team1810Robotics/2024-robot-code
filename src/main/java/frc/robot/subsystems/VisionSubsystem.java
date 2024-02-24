package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    Transform3d robotToCam = VisionConstants.CAMERA_OFFSET;

    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    PhotonPipelineResult result;

    public VisionSubsystem() {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
    }

    /** @return whether or not an AprilTag is detected */
    public boolean hasTarget() {
        return result.hasTargets();
    }

    /** (don't use this unless you're desperate)
     * @return the PhotonCamera object*/
    public PhotonCamera getCamera() {
        return camera;
    }

    /** @return the best target's ID */
    public int getTargetId() {
        return result.getBestTarget().getFiducialId();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return result.getTargets();
    }

    /** @return the distance from the target in meters */
    public double getPoseFromTarget() {
        PhotonTrackedTarget target = getTargets().get(0);
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            robotToCam.getZ(),
            target.getBestCameraToTarget().getZ(),
            robotToCam.getRotation().getZ(),
            Units.degreesToRadians(target.getPitch()));

        return range;
    }

    /** @returns the vision pipeline's result (all of its data) */
    public PhotonPipelineResult getResult() {
        return result;
    }
    
    /** WARNING: Currently causes a loop overrun, do not use
     * @return a Pose3d representing the robot's position on the field*/
    public Pose3d getRobotPose() {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        return pose.get().estimatedPose;
    }

    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }
}