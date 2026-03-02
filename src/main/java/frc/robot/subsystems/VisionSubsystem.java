package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
  private final PhotonCamera m_ForwardCamera;
  private final PhotonPoseEstimator m_ForwardPoseEstimator;
  private final PhotonCamera m_ReverseCamera;
  private final PhotonPoseEstimator m_ReversePoseEstimator;
  private final AprilTagFieldLayout m_FieldLayout;

  private Optional<VisionMeasurement> m_LatestForwardVisionMeasurement = Optional.empty();
  private Optional<VisionMeasurement> m_LatestReverseVisionMeasurement = Optional.empty();

  public VisionSubsystem() {
    m_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_ForwardCamera = new PhotonCamera(Constants.Vision.kForwardCameraName);
    m_ForwardPoseEstimator =
        new PhotonPoseEstimator(m_FieldLayout, Constants.Vision.kForwardRobotToCam3d);
    m_ReverseCamera = new PhotonCamera(Constants.Vision.kReverseCameraName);
    m_ReversePoseEstimator =
        new PhotonPoseEstimator(m_FieldLayout, Constants.Vision.kReverseRobotToCam3d);
  }

  public Optional<VisionMeasurement> getLatestForwardVisionMeasurement() {
    return m_LatestForwardVisionMeasurement;
  }

  public Optional<VisionMeasurement> getLatestReverseVisionMeasurement() {
    return m_LatestReverseVisionMeasurement;
  }

  @Override
  public void periodic() {
    m_LatestForwardVisionMeasurement = processCamera(m_ForwardCamera, m_ForwardPoseEstimator);
    m_LatestReverseVisionMeasurement = processCamera(m_ReverseCamera, m_ReversePoseEstimator);

    publishMeasurementTelemetry("Forward", m_LatestForwardVisionMeasurement);
    publishMeasurementTelemetry("Reverse", m_LatestReverseVisionMeasurement);
  }

  private void publishMeasurementTelemetry(
      String cameraName, Optional<VisionMeasurement> measurement) {
    String prefix = "Vision/" + cameraName + "/";
    SmartDashboard.putBoolean(prefix + "Accepted", measurement.isPresent());
    measurement.ifPresent(
        m -> {
          SmartDashboard.putNumber(prefix + "PoseX_m", m.pose().getX());
          SmartDashboard.putNumber(prefix + "PoseY_m", m.pose().getY());
          SmartDashboard.putNumber(prefix + "PoseHeading_deg", m.pose().getRotation().getDegrees());
          SmartDashboard.putNumber(prefix + "StdDevXY", m.stdDevs().get(0, 0));
          SmartDashboard.putNumber(
              prefix + "StdDevHeading_deg", Math.toDegrees(m.stdDevs().get(2, 0)));
        });
  }

  private Optional<VisionMeasurement> processCamera(
      PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
    Optional<VisionMeasurement> measurement = Optional.empty();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;

      int targetCount = result.targets.size();
      if (targetCount < Constants.Vision.kMinAprilTagsForPose) continue;
      if (targetCount == 1
          && result.getBestTarget().getPoseAmbiguity()
              > Constants.Vision.kMaxAcceptableSingleTagAmbiguity) continue;

      // Single-tag distance limit: PnP (Perspective-n-Point) becomes unreliable at range
      if (targetCount == 1) {
        double distanceToTag =
            result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        if (distanceToTag > Constants.Vision.kMaxSingleTagDistanceMeters) continue;
      }
      // Fallback strategy: always prefer multi-tag when available to solve ambiguity problem
      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
      if (estimatedPose.isEmpty())
        estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
      if (estimatedPose.isEmpty()) continue;

      // Z-height sanity: robot must be near the floor
      if (Math.abs(estimatedPose.get().estimatedPose.getZ())
          > Constants.Vision.kMaxPoseHeightMeters) continue;

      // Field boundary: reject poses outside the field (plus a small margin)
      Pose2d robotPose = estimatedPose.get().estimatedPose.toPose2d();
      double fieldMargin = Constants.Vision.kFieldBoundaryMarginMeters;
      if (robotPose.getX() < -fieldMargin
          || robotPose.getX() > m_FieldLayout.getFieldLength() + fieldMargin
          || robotPose.getY() < -fieldMargin
          || robotPose.getY() > m_FieldLayout.getFieldWidth() + fieldMargin) continue;

      measurement =
          Optional.of(
              new VisionMeasurement(
                  robotPose,
                  estimatedPose.get().timestampSeconds,
                  computeDynamicStdDevs(estimatedPose.get())));
    }

    return measurement;
  }

  private static Matrix<N3, N1> computeDynamicStdDevs(EstimatedRobotPose estimatedPose) {
    boolean isMultiTag = estimatedPose.targetsUsed.size() > 1;

    double totalDistance = 0.0;
    for (var target : estimatedPose.targetsUsed) {
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    double avgDistance = totalDistance / estimatedPose.targetsUsed.size();

    double baseXY;
    double baseHeading;
    double exponent;
    if (isMultiTag) {
      baseXY = Constants.Vision.kMultiTagBaseXYStdDev;
      baseHeading = Constants.Vision.kMultiTagBaseHeadingStdDev;
      exponent = Constants.Vision.kMultiTagDistanceExponent;
    } else {
      baseXY = Constants.Vision.kSingleTagBaseXYStdDev;
      baseHeading = Constants.Vision.kSingleTagBaseHeadingStdDev;
      exponent = Constants.Vision.kSingleTagDistanceExponent;
    }

    double distanceFactor = Math.pow(avgDistance, exponent);
    double xyStdDev = baseXY * distanceFactor;
    double headingStdDev = baseHeading * distanceFactor;

    return VecBuilder.fill(xyStdDev, xyStdDev, headingStdDev);
  }

  public static record VisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
}
