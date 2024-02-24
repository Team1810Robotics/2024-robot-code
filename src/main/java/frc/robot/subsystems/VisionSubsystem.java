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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    
    
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
    
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

    PhotonPipelineResult result;

    public VisionSubsystem() {
        /* SmartDashboard.putBoolean("has target", hasTarget());
        SmartDashboard.putNumber("best target ID", getTargetId()); */
    }

    
    public boolean hasTarget() {
        return result.hasTargets();
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public int getTargetId() {
        return result.getBestTarget().getFiducialId();
    }

    public List<PhotonTrackedTarget> getTargets() {
        return result.getTargets();
    }

    public double getPoseFromTarget() {
        PhotonTrackedTarget target = getTargets().get(0);
        double range = PhotonUtils.calculateDistanceToTargetMeters(
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

    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }

    public PhotonPipelineResult getResult() {
        return result;
    }

    public Pose3d getRobotPose() {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        return pose.get().estimatedPose;
    }
}