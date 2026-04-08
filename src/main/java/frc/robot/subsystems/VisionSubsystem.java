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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
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
  private final PhotonCamera m_ReverseCamera;
  private final PhotonPoseEstimator m_ReversePoseEstimator;
  private final AprilTagFieldLayout m_FieldLayout;

  // Queued measurements from every processed frame since the last drain.
  private final List<VisionMeasurement> m_RightMeasurements = new ArrayList<>();
  private final List<VisionMeasurement> m_ForwardMeasurements = new ArrayList<>();
  private final List<VisionMeasurement> m_ReverseMeasurements = new ArrayList<>();

  // Last accepted measurement per camera — purely for telemetry / logVisionMeasurement.
  private Optional<VisionMeasurement> m_LastRightAccepted = Optional.empty();
  private Optional<VisionMeasurement> m_LastForwardAccepted = Optional.empty();
  private Optional<VisionMeasurement> m_LastReverseAccepted = Optional.empty();

  // Edge-triggered calibration warnings so the log isn't spammed every loop.
  private boolean m_RightUncalibratedLogged = false;
  private boolean m_ForwardUncalibratedLogged = false;
  private boolean m_ReverseUncalibratedLogged = false;

  private int m_RightRejectionCount = 0;
  private int m_ForwardRejectionCount = 0;
  private int m_ReverseRejectionCount = 0;

  // Track rejection reasons for logging
  private String m_RightRejectionReason = "";
  private String m_ForwardRejectionReason = "";
  private String m_ReverseRejectionReason = "";

  private final StructPublisher<Pose2d> m_RightPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Vision/Right/Pose", Pose2d.struct)
          .publish();
  private final StructPublisher<Pose2d> m_ForwardPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Vision/Forward/Pose", Pose2d.struct)
          .publish();
  private final StructPublisher<Pose2d> m_ReversePosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Vision/Reverse/Pose", Pose2d.struct)
          .publish();

  // Simulation support
  private VisionSystemSim m_visionSim;
  private Pose2d m_simRobotPose = new Pose2d();

  // Logging - throttle to every 10th cycle (200ms)
  private int m_logCounter = 0;

  public VisionSubsystem() {
    m_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_RightCamera = new PhotonCamera(Constants.Vision.kRightCameraName);
    m_RightPoseEstimator =
        new PhotonPoseEstimator(m_FieldLayout, Constants.Vision.kRightRobotToCam3d);
    m_ForwardCamera = new PhotonCamera(Constants.Vision.kForwardCameraName);
    m_ForwardPoseEstimator =
        new PhotonPoseEstimator(m_FieldLayout, Constants.Vision.kForwardRobotToCam3d);
    m_ReverseCamera = new PhotonCamera(Constants.Vision.kReverseCameraName);
    m_ReversePoseEstimator =
        new PhotonPoseEstimator(m_FieldLayout, Constants.Vision.kReverseRobotToCam3d);

    SmartDashboard.putBoolean("Vision/VerboseTelemetry", false);

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
      var reverseCameraSim = new PhotonCameraSim(m_ReverseCamera, cameraProp);
      rightCameraSim.enableDrawWireframe(true);
      forwardCameraSim.enableDrawWireframe(true);
      reverseCameraSim.enableDrawWireframe(true);
      rightCameraSim.setMaxSightRange(Constants.Vision.kSimMaxSightRangeMeters);
      forwardCameraSim.setMaxSightRange(Constants.Vision.kSimMaxSightRangeMeters);
      reverseCameraSim.setMaxSightRange(Constants.Vision.kSimMaxSightRangeMeters);

      m_visionSim.addCamera(rightCameraSim, Constants.Vision.kRightRobotToCam3d);
      m_visionSim.addCamera(forwardCameraSim, Constants.Vision.kForwardRobotToCam3d);
      m_visionSim.addCamera(reverseCameraSim, Constants.Vision.kReverseRobotToCam3d);
    }
  }

  /**
   * Returns and clears all vision measurements queued since the last drain. Each call returns a
   * snapshot — repeated calls without new frames return empty lists.
   */
  public List<VisionMeasurement> drainRightMeasurements() {
    return drain(m_RightMeasurements);
  }

  public List<VisionMeasurement> drainForwardMeasurements() {
    return drain(m_ForwardMeasurements);
  }

  public List<VisionMeasurement> drainReverseMeasurements() {
    return drain(m_ReverseMeasurements);
  }

  private static List<VisionMeasurement> drain(List<VisionMeasurement> src) {
    if (src.isEmpty()) {
      return List.of();
    }
    var copy = new ArrayList<>(src);
    src.clear();
    return copy;
  }

  @Override
  public void periodic() {
    boolean verbose = SmartDashboard.getBoolean("Vision/VerboseTelemetry", true);

    if (verbose) {
      SmartDashboard.putBoolean("Vision/Right/Connected", m_RightCamera.isConnected());
      SmartDashboard.putBoolean("Vision/Forward/Connected", m_ForwardCamera.isConnected());
      SmartDashboard.putBoolean("Vision/Reverse/Connected", m_ReverseCamera.isConnected());
    }

    processAllResults(m_RightCamera, m_RightPoseEstimator, "Right", m_RightMeasurements, verbose);
    processAllResults(
        m_ForwardCamera, m_ForwardPoseEstimator, "Forward", m_ForwardMeasurements, verbose);
    processAllResults(
        m_ReverseCamera, m_ReversePoseEstimator, "Reverse", m_ReverseMeasurements, verbose);

    // Log critical vision data every 10th cycle (200ms) for match analysis
    if (++m_logCounter >= 10) {
      logVisionMeasurement(
          "Right", m_LastRightAccepted, m_RightCamera.isConnected(), m_RightRejectionReason);
      logVisionMeasurement(
          "Forward",
          m_LastForwardAccepted,
          m_ForwardCamera.isConnected(),
          m_ForwardRejectionReason);
      logVisionMeasurement(
          "Reverse",
          m_LastReverseAccepted,
          m_ReverseCamera.isConnected(),
          m_ReverseRejectionReason);
      m_logCounter = 0;
    }
  }

  /**
   * Processes every unread pipeline result for a camera (not just the latest), applies the full
   * filter cascade per frame, and appends accepted measurements to {@code outList}. Also verifies
   * that PhotonVision has returned a calibration matrix — if not, we refuse to compute poses at all
   * (they'd be garbage) and log an edge-triggered warning.
   */
  private void processAllResults(
      PhotonCamera camera,
      PhotonPoseEstimator poseEstimator,
      String cameraName,
      List<VisionMeasurement> outList,
      boolean verbose) {
    if (!camera.isConnected()) {
      // Don't feed stale measurements from a disconnected camera
      outList.clear();
      clearLastAccepted(cameraName);
      return;
    }

    // Calibration verification — edge-triggered so the log isn't spammed.
    boolean calibrated = camera.getCameraMatrix().isPresent();
    trackCalibrationState(cameraName, calibrated);
    if (verbose) {
      SmartDashboard.putBoolean("Vision/" + cameraName + "/Calibrated", calibrated);
    }
    if (!calibrated) {
      // Without intrinsics the pose solution is meaningless — don't even try.
      return;
    }

    var allResults = camera.getAllUnreadResults();
    if (allResults.isEmpty()) {
      return;
    }

    for (var result : allResults) {
      VisionMeasurement measurement = processFrame(result, poseEstimator, cameraName, verbose);
      if (measurement != null) {
        outList.add(measurement);
        publishMeasurementTelemetry(cameraName, Optional.of(measurement), verbose);
        switch (cameraName) {
          case "Right" -> m_LastRightAccepted = Optional.of(measurement);
          case "Forward" -> m_LastForwardAccepted = Optional.of(measurement);
          case "Reverse" -> m_LastReverseAccepted = Optional.of(measurement);
          default -> {}
        }
      }
    }
  }

  private void clearLastAccepted(String cameraName) {
    switch (cameraName) {
      case "Right" -> m_LastRightAccepted = Optional.empty();
      case "Forward" -> m_LastForwardAccepted = Optional.empty();
      case "Reverse" -> m_LastReverseAccepted = Optional.empty();
      default -> {}
    }
  }

  private void trackCalibrationState(String cameraName, boolean calibrated) {
    boolean previouslyLogged =
        switch (cameraName) {
          case "Right" -> m_RightUncalibratedLogged;
          case "Forward" -> m_ForwardUncalibratedLogged;
          case "Reverse" -> m_ReverseUncalibratedLogged;
          default -> false;
        };

    if (!calibrated && !previouslyLogged) {
      DataLogManager.log(
          "Vision/" + cameraName + " WARNING: camera returned no calibration matrix");
      setUncalibratedLogged(cameraName, true);
    } else if (calibrated && previouslyLogged) {
      DataLogManager.log("Vision/" + cameraName + " calibration now available");
      setUncalibratedLogged(cameraName, false);
    }
  }

  private void setUncalibratedLogged(String cameraName, boolean value) {
    switch (cameraName) {
      case "Right" -> m_RightUncalibratedLogged = value;
      case "Forward" -> m_ForwardUncalibratedLogged = value;
      case "Reverse" -> m_ReverseUncalibratedLogged = value;
      default -> {}
    }
  }

  private void logVisionMeasurement(
      String camera,
      Optional<VisionMeasurement> measurement,
      boolean connected,
      String rejectionReason) {
    if (!connected) {
      DataLogManager.log("Vision/" + camera + " disconnected");
      return;
    }
    if (measurement.isEmpty()) {
      DataLogManager.log("Vision/" + camera + " rejected:" + rejectionReason);
      return;
    }
    VisionMeasurement m = measurement.get();
    DataLogManager.log(
        "Vision/"
            + camera
            + " t:"
            + String.format("%.3f", m.timestampSeconds())
            + " x:"
            + String.format("%.2f", m.pose().getX())
            + " y:"
            + String.format("%.2f", m.pose().getY())
            + " h:"
            + String.format("%.1f", m.pose().getRotation().getDegrees())
            + " stdXY:"
            + String.format("%.3f", m.stdDevs().get(0, 0))
            + " stdH:"
            + String.format("%.2f", Math.toDegrees(m.stdDevs().get(2, 0))));
  }

  private void publishMeasurementTelemetry(
      String cameraName, Optional<VisionMeasurement> measurement, boolean verbose) {
    if (measurement.isPresent()) {
      VisionMeasurement m = measurement.get();
      switch (cameraName) {
        case "Right" -> m_RightPosePublisher.set(m.pose());
        case "Forward" -> m_ForwardPosePublisher.set(m.pose());
        case "Reverse" -> m_ReversePosePublisher.set(m.pose());
      }

      if (verbose) {
        String prefix = "Vision/" + cameraName + "/";
        SmartDashboard.putBoolean(prefix + "Accepted", true);
        SmartDashboard.putString(prefix + "RejectionReason", "");
        SmartDashboard.putNumber(prefix + "PoseX_m", m.pose().getX());
        SmartDashboard.putNumber(prefix + "PoseY_m", m.pose().getY());
        SmartDashboard.putNumber(prefix + "PoseHeading_deg", m.pose().getRotation().getDegrees());
        SmartDashboard.putNumber(prefix + "StdDevXY", m.stdDevs().get(0, 0));
        SmartDashboard.putNumber(
            prefix + "StdDevHeading_deg", Math.toDegrees(m.stdDevs().get(2, 0)));
      }
    } else if (verbose) {
      SmartDashboard.putBoolean("Vision/" + cameraName + "/Accepted", false);
    }
  }

  private void publishRejection(String cameraName, String reason, boolean verbose) {
    // Store rejection reason for logging
    if (cameraName.equals("Right")) {
      m_RightRejectionReason = reason;
      if (verbose) {
        SmartDashboard.putString("Vision/" + cameraName + "/RejectionReason", reason);
        SmartDashboard.putNumber(
            "Vision/" + cameraName + "/RejectionCount", ++m_RightRejectionCount);
      }
    } else if (cameraName.equals("Forward")) {
      m_ForwardRejectionReason = reason;
      if (verbose) {
        SmartDashboard.putString("Vision/" + cameraName + "/RejectionReason", reason);
        SmartDashboard.putNumber(
            "Vision/" + cameraName + "/RejectionCount", ++m_ForwardRejectionCount);
      }
    } else {
      m_ReverseRejectionReason = reason;
      if (verbose) {
        SmartDashboard.putString("Vision/" + cameraName + "/RejectionReason", reason);
        SmartDashboard.putNumber(
            "Vision/" + cameraName + "/RejectionCount", ++m_ReverseRejectionCount);
      }
    }
  }

  /**
   * Applies the full filter cascade to a single PhotonVision pipeline result. Returns the accepted
   * {@link VisionMeasurement} or {@code null} if the frame was rejected.
   */
  private VisionMeasurement processFrame(
      PhotonPipelineResult result,
      PhotonPoseEstimator poseEstimator,
      String cameraName,
      boolean verbose) {
    if (!result.hasTargets()) {
      publishRejection(cameraName, "no_targets", verbose);
      if (verbose) {
        SmartDashboard.putBoolean("Vision/" + cameraName + "/TagDetected", false);
      }
      return null;
    }

    if (verbose) {
      String prefix = "Vision/" + cameraName + "/";
      SmartDashboard.putBoolean(prefix + "TagDetected", true);
      double[] visibleIds = result.targets.stream().mapToDouble(t -> t.getFiducialId()).toArray();
      SmartDashboard.putNumberArray(prefix + "VisibleTagIDs", visibleIds);
      double[] ambiguities =
          result.targets.stream().mapToDouble(t -> t.getPoseAmbiguity()).toArray();
      SmartDashboard.putNumberArray(prefix + "Ambiguities", ambiguities);
    }

    int targetCount = result.targets.size();

    if (targetCount < Constants.Vision.kMinAprilTagsForPose) {
      publishRejection(cameraName, "too_few_targets:" + targetCount, verbose);
      return null;
    }

    if (targetCount == 1
        && result.getBestTarget().getPoseAmbiguity()
            > Constants.Vision.kMaxAcceptableSingleTagAmbiguity) {
      publishRejection(
          cameraName,
          "ambiguity_too_high:" + String.format("%.2f", result.getBestTarget().getPoseAmbiguity()),
          verbose);
      return null;
    }

    if (targetCount == 1) {
      double distanceToTag =
          result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
      if (distanceToTag > Constants.Vision.kMaxSingleTagDistanceMeters) {
        publishRejection(
            cameraName, "single_tag_too_far:" + String.format("%.2f", distanceToTag), verbose);
        return null;
      }
    }

    // Attempt coprocessor multi-tag first, fall back to lowest ambiguity
    Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
    String multiTagFailReason = "";

    if (estimatedPose.isEmpty()) {
      if (verbose) {
        // Diagnose why coprocessor multi-tag failed.
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
      }

      estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
    }

    if (estimatedPose.isEmpty()) {
      if (verbose) {
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
            "pose_estimation_failed|multitag:" + multiTagFailReason + "|fallback:" + fallbackReason,
            verbose);
      }
      return null;
    }

    double poseZ = estimatedPose.get().estimatedPose.getZ();
    if (Math.abs(poseZ) > Constants.Vision.kMaxPoseHeightMeters) {
      publishRejection(cameraName, "z_out_of_range:" + String.format("%.2f", poseZ), verbose);
      return null;
    }

    Pose2d robotPose = estimatedPose.get().estimatedPose.toPose2d();
    double fieldMargin = Constants.Vision.kFieldBoundaryMarginMeters;
    if (robotPose.getX() < -fieldMargin
        || robotPose.getX() > m_FieldLayout.getFieldLength() + fieldMargin
        || robotPose.getY() < -fieldMargin
        || robotPose.getY() > m_FieldLayout.getFieldWidth() + fieldMargin) {
      publishRejection(
          cameraName,
          "outside_field:" + String.format("(%.2f, %.2f)", robotPose.getX(), robotPose.getY()),
          verbose);
      return null;
    }

    Matrix<N3, N1> stdDevs = computeDynamicStdDevs(estimatedPose.get());

    if (verbose) {
      double[] usedIds =
          estimatedPose.get().targetsUsed.stream().mapToDouble(t -> t.getFiducialId()).toArray();
      SmartDashboard.putNumberArray("Vision/" + cameraName + "/UsedTagIDs", usedIds);
    }

    return new VisionMeasurement(robotPose, estimatedPose.get().timestampSeconds, stdDevs);
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
