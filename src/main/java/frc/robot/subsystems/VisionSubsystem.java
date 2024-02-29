package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private double previousPipelineTimestamp = 0.0;

    private PhotonCamera photonCamera;
    private SwerveSubsystem drive;

    private Field2d field = new Field2d();

    public VisionSubsystem(SwerveSubsystem drive) {
        this.drive = drive;

        this.photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();

        if (resultTimestamp == previousPipelineTimestamp && !pipelineResult.hasTargets()) return;
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
