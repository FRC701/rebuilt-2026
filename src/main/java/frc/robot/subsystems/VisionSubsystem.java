package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera m_RightCamera;
  private final PhotonPoseEstimator m_RightPoseEstimator;
  private final PhotonCamera m_ForwardCamera;
  private final PhotonPoseEstimator m_ForwardPoseEstimator;
  private final AprilTagFieldLayout m_FieldLayout;

  private Optional<VisionMeasurement> m_LatestRightVisionMeasurement = Optional.empty();
  private Optional<VisionMeasurement> m_LatestForwardVisionMeasurement = Optional.empty();

  private int m_RightRejectionCount = 0;
  private int m_ForwardRejectionCount = 0;

  // Simulation support
  private VisionSystemSim m_visionSim;
  private Pose2d m_simRobotPose = new Pose2d();

  public VisionSubsystem() {
    m_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_RightCamera = new PhotonCamera(Constants.Vision.kRightCameraName);
    m_RightPoseEstimator =
        new PhotonPoseEstimator(m_FieldLayout, Constants.Vision.kRightRobotToCam3d);
    m_ForwardCamera = new PhotonCamera(Constants.Vision.kForwardCameraName);
    m_ForwardPoseEstimator =
        new PhotonPoseEstimator(m_FieldLayout, Constants.Vision.kForwardRobotToCam3d);

    if (Utils.isSimulation()) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(m_FieldLayout);

      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(
          Constants.Vision.kSimCameraResWidth,
          Constants.Vision.kSimCameraResHeight,
          Rotation2d.fromDegrees(Constants.Vision.kSimCameraFOVDeg));
      cameraProp.setCalibError(0.25, 0.08);
      cameraProp.setFPS(Constants.Vision.kSimCameraFPS);
      cameraProp.setAvgLatencyMs(Constants.Vision.kSimAvgLatencyMs);
      cameraProp.setLatencyStdDevMs(Constants.Vision.kSimLatencyStdDevMs);

      var rightCameraSim = new PhotonCameraSim(m_RightCamera, cameraProp);
      var forwardCameraSim = new PhotonCameraSim(m_ForwardCamera, cameraProp);
      rightCameraSim.enableDrawWireframe(true);
      forwardCameraSim.enableDrawWireframe(true);
      rightCameraSim.setMaxSightRange(Constants.Vision.kSimMaxSightRangeMeters);
      forwardCameraSim.setMaxSightRange(Constants.Vision.kSimMaxSightRangeMeters);

      m_visionSim.addCamera(rightCameraSim, Constants.Vision.kRightRobotToCam3d);
      m_visionSim.addCamera(forwardCameraSim, Constants.Vision.kForwardRobotToCam3d);
    }
  }

  public Optional<VisionMeasurement> getLatestRightVisionMeasurement() {
    return m_LatestRightVisionMeasurement;
  }

  public Optional<VisionMeasurement> getLatestForwardVisionMeasurement() {
    return m_LatestForwardVisionMeasurement;
  }

  @Override
  public void periodic() {
    boolean rightConnected = m_RightCamera.isConnected();
    boolean forwardConnected = m_ForwardCamera.isConnected();

    SmartDashboard.putBoolean("Vision/Right/Connected", rightConnected);
    SmartDashboard.putBoolean("Vision/Forward/Connected", forwardConnected);

    m_LatestRightVisionMeasurement =
        rightConnected
            ? processCamera(m_RightCamera, m_RightPoseEstimator, "Right")
            : Optional.empty();
    m_LatestForwardVisionMeasurement =
        forwardConnected
            ? processCamera(m_ForwardCamera, m_ForwardPoseEstimator, "Forward")
            : Optional.empty();

    publishMeasurementTelemetry("Right", m_LatestRightVisionMeasurement);
    publishMeasurementTelemetry("Forward", m_LatestForwardVisionMeasurement);
  }

  private void publishMeasurementTelemetry(
      String cameraName, Optional<VisionMeasurement> measurement) {
    String prefix = "Vision/" + cameraName + "/";
    SmartDashboard.putBoolean(prefix + "Accepted", measurement.isPresent());

    if (measurement.isPresent()) {
      // Clear rejection reason when a measurement is accepted
      SmartDashboard.putString(prefix + "RejectionReason", "");
      VisionMeasurement m = measurement.get();
      SmartDashboard.putNumber(prefix + "PoseX_m", m.pose().getX());
      SmartDashboard.putNumber(prefix + "PoseY_m", m.pose().getY());
      SmartDashboard.putNumber(prefix + "PoseHeading_deg", m.pose().getRotation().getDegrees());
      SmartDashboard.putNumber(prefix + "StdDevXY", m.stdDevs().get(0, 0));
      SmartDashboard.putNumber(prefix + "StdDevHeading_deg", Math.toDegrees(m.stdDevs().get(2, 0)));
    }
  }

  private void publishRejection(String cameraName, String reason) {
    String prefix = "Vision/" + cameraName + "/";
    SmartDashboard.putString(prefix + "RejectionReason", reason);
    if (cameraName.equals("Right")) {
      SmartDashboard.putNumber(prefix + "RejectionCount", ++m_RightRejectionCount);
    } else {
      SmartDashboard.putNumber(prefix + "RejectionCount", ++m_ForwardRejectionCount);
    }
  }

  private Optional<VisionMeasurement> processCamera(
      PhotonCamera camera, PhotonPoseEstimator poseEstimator, String cameraName) {
    Optional<VisionMeasurement> measurement = Optional.empty();
    String prefix = "Vision/" + cameraName + "/";
    boolean tagDetected = false;

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) {
        publishRejection(cameraName, "no_targets");
        continue;
      }

      tagDetected = true;

      // Publish all tag IDs visible in this frame
      double[] visibleIds = result.targets.stream().mapToDouble(t -> t.getFiducialId()).toArray();
      SmartDashboard.putNumberArray(prefix + "VisibleTagIDs", visibleIds);
      double[] ambiguities =
          result.targets.stream().mapToDouble(t -> t.getPoseAmbiguity()).toArray();
      SmartDashboard.putNumberArray(prefix + "Ambiguities", ambiguities);

      int targetCount = result.targets.size();

      if (targetCount < Constants.Vision.kMinAprilTagsForPose) {
        publishRejection(cameraName, "too_few_targets:" + targetCount);
        continue;
      }

      if (targetCount == 1
          && result.getBestTarget().getPoseAmbiguity()
              > Constants.Vision.kMaxAcceptableSingleTagAmbiguity) {
        publishRejection(
            cameraName,
            "ambiguity_too_high:"
                + String.format("%.2f", result.getBestTarget().getPoseAmbiguity()));
        continue;
      }

      if (targetCount == 1) {
        double distanceToTag =
            result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        if (distanceToTag > Constants.Vision.kMaxSingleTagDistanceMeters) {
          publishRejection(
              cameraName, "single_tag_too_far:" + String.format("%.2f", distanceToTag));
          continue;
        }
      }

      // Attempt coprocessor multi-tag first, fall back to lowest ambiguity
      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
      String multiTagFailReason = "";

      if (estimatedPose.isEmpty()) {
        // Diagnose why coprocessor multi-tag failed.
        // PnpResult is always a plain object (not Optional) -- validity is indicated by
        // bestReprojErr being non-zero. A value of 0 means the default/invalid state.
        var multiTagResult = result.getMultiTagResult();
        if (multiTagResult.isEmpty()) {
          multiTagFailReason = "no_multitag_result";
        } else if (multiTagResult.get().estimatedPose.bestReprojErr == 0) {
          multiTagFailReason = "multitag_transform_invalid(reproj=0)";
        } else {
          multiTagFailReason =
              "multitag_estimator_rejected(reproj:"
                  + String.format("%.2f", multiTagResult.get().estimatedPose.bestReprojErr)
                  + "px)";
        }

        estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (estimatedPose.isEmpty()) {
        // Diagnose why lowest-ambiguity fallback also failed
        var bestTarget = result.getBestTarget();
        int tagId = bestTarget.getFiducialId();
        boolean tagInLayout = m_FieldLayout.getTagPose(tagId).isPresent();
        double ambiguity = bestTarget.getPoseAmbiguity();
        SmartDashboard.putNumber("Vision/" + cameraName + "/ambiguity", ambiguity);

        String fallbackReason;
        if (!tagInLayout) {
          fallbackReason = "tag_" + tagId + "_not_in_layout";
        } else if (ambiguity < 0) {
          fallbackReason = "ambiguity_unavailable(tag_" + tagId + ")";
        } else {
          fallbackReason = "lowest_ambiguity_rejected:" + String.format("%.2f", ambiguity);
        }

        publishRejection(
            cameraName,
            "pose_estimation_failed|multitag:"
                + multiTagFailReason
                + "|fallback:"
                + fallbackReason);
        continue;
      }

      double poseZ = estimatedPose.get().estimatedPose.getZ();
      if (Math.abs(poseZ) > Constants.Vision.kMaxPoseHeightMeters) {
        publishRejection(cameraName, "z_out_of_range:" + String.format("%.2f", poseZ));
        continue;
      }

      Pose2d robotPose = estimatedPose.get().estimatedPose.toPose2d();
      double fieldMargin = Constants.Vision.kFieldBoundaryMarginMeters;
      if (robotPose.getX() < -fieldMargin
          || robotPose.getX() > m_FieldLayout.getFieldLength() + fieldMargin
          || robotPose.getY() < -fieldMargin
          || robotPose.getY() > m_FieldLayout.getFieldWidth() + fieldMargin) {
        publishRejection(
            cameraName,
            "outside_field:" + String.format("(%.2f, %.2f)", robotPose.getX(), robotPose.getY()));
        continue;
      }

      Matrix<N3, N1> stdDevs = computeDynamicStdDevs(estimatedPose.get());

      // Publish the tag IDs that actually contributed to this accepted pose estimate
      double[] usedIds =
          estimatedPose.get().targetsUsed.stream().mapToDouble(t -> t.getFiducialId()).toArray();
      SmartDashboard.putNumberArray(prefix + "UsedTagIDs", usedIds);

      measurement =
          Optional.of(
              new VisionMeasurement(robotPose, estimatedPose.get().timestampSeconds, stdDevs));
    }

    // Published once per periodic cycle, reflects whether any queued frame had visible tags
    SmartDashboard.putBoolean(prefix + "TagDetected", tagDetected);

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

  public void setSimRobotPose(Pose2d pose) {
    m_simRobotPose = pose;
  }

  @Override
  public void simulationPeriodic() {
    if (m_visionSim != null) {
      m_visionSim.update(m_simRobotPose);
    }
  }

  public static record VisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}
}
