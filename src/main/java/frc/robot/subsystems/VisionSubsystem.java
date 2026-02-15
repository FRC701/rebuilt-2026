package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_poseEstimator;
  private final AprilTagFieldLayout m_fieldLayout;

  private Optional<VisionMeasurement> m_latestMeasurement = Optional.empty();

  public VisionSubsystem() {
    m_camera = new PhotonCamera(Constants.Vision.kcameraName);
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, Constants.Vision.kRobotToCam3d);
  }

  public Optional<VisionMeasurement> getLatestMeasurement() {
    return m_latestMeasurement;
  }

  @Override
  public void periodic() {
    m_latestMeasurement = Optional.empty();

    for (PhotonPipelineResult result : m_camera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;
      if (result.targets.size() < Constants.Vision.kMinAprilTagsForPose) continue;
      if (result.targets.size() == 1
          && result.getBestTarget().getPoseAmbiguity()
              > Constants.Vision.kMaxAcceptableSingleTagAmbiguity) continue;

      // Single-tag distance limit: PnP becomes unreliable at range
      if (result.targets.size() == 1) {
        double dist = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        if (dist > Constants.Vision.kMaxSingleTagDistanceMeters) continue;
      }

      Optional<EstimatedRobotPose> estimate = m_poseEstimator.estimateCoprocMultiTagPose(result);
      if (estimate.isEmpty()) estimate = m_poseEstimator.estimateLowestAmbiguityPose(result);
      if (estimate.isEmpty()) continue;

      // Z-height sanity: robot must be near the floor
      if (Math.abs(estimate.get().estimatedPose.getZ()) > Constants.Vision.kMaxPoseHeightMeters)
        continue;

      // Field boundary: reject poses outside the field (plus a small margin)
      Pose2d p = estimate.get().estimatedPose.toPose2d();
      double margin = Constants.Vision.kFieldBoundaryMarginMeters;
      if (p.getX() < -margin
          || p.getX() > m_fieldLayout.getFieldLength() + margin
          || p.getY() < -margin
          || p.getY() > m_fieldLayout.getFieldWidth() + margin) continue;

      m_latestMeasurement =
          Optional.of(
              new VisionMeasurement(
                  p, estimate.get().timestampSeconds, Constants.Vision.kVisionStdDevs));
    }

    SmartDashboard.putBoolean("Vision/Accepted", m_latestMeasurement.isPresent());
    m_latestMeasurement.ifPresent(
        m -> {
          SmartDashboard.putNumber("Vision/PoseX_m", m.pose().getX());
          SmartDashboard.putNumber("Vision/PoseY_m", m.pose().getY());
          SmartDashboard.putNumber("Vision/PoseHeading_deg", m.pose().getRotation().getDegrees());
        });
  }

  public static record VisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
}
