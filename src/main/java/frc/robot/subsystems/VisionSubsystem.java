package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
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

  /** Maximum queued measurements per camera before oldest entries are dropped. */
  private static final int kMaxQueuedMeasurements = 20;

  /** All per-camera state in one place — eliminates the 3x field duplication. */
  private static class CameraState {
    final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;
    final String name;
    final Transform3d robotToCam;
    final StructPublisher<Pose2d> posePublisher;
    final List<VisionMeasurement> measurements = new ArrayList<>();
    Optional<VisionMeasurement> lastAccepted = Optional.empty();
    boolean uncalibratedLogged = false;
    boolean calibrationConfirmed = false;
    int rejectionCount = 0;
    String rejectionReason = "";

    CameraState(
        String name,
        String cameraConfigName,
        AprilTagFieldLayout fieldLayout,
        Transform3d robotToCam) {
      this.name = name;
      this.robotToCam = robotToCam;
      this.camera = new PhotonCamera(cameraConfigName);
      this.poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
      this.posePublisher =
          NetworkTableInstance.getDefault()
              .getStructTopic("Vision/" + name + "/Pose", Pose2d.struct)
              .publish();
    }
  }

  private final AprilTagFieldLayout m_FieldLayout;
  private final CameraState[] m_cameras;

  private final NetworkTable m_visionTable = NetworkTableInstance.getDefault().getTable("Vision");

  // Simulation support
  private VisionSystemSim m_visionSim;
  private Pose2d m_simRobotPose = new Pose2d();

  // Logging — throttle to every 10th cycle (200ms)
  private int m_logCounter = 0;

  public VisionSubsystem() {
    m_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    m_cameras =
        new CameraState[] {
          new CameraState(
              "Right",
              Constants.Vision.kRightCameraName,
              m_FieldLayout,
              Constants.Vision.kRightRobotToCam3d),
          new CameraState(
              "Forward",
              Constants.Vision.kForwardCameraName,
              m_FieldLayout,
              Constants.Vision.kForwardRobotToCam3d),
          new CameraState(
              "Reverse",
              Constants.Vision.kReverseCameraName,
              m_FieldLayout,
              Constants.Vision.kReverseRobotToCam3d),
        };

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

      for (var cam : m_cameras) {
        var cameraSim = new PhotonCameraSim(cam.camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
        cameraSim.setMaxSightRange(Constants.Vision.kSimMaxSightRangeMeters);
        m_visionSim.addCamera(cameraSim, cam.robotToCam);
      }
    }
  }

  /**
   * Returns and clears all vision measurements queued across all cameras since the last drain. Each
   * {@link VisionMeasurement} includes a camera name for logging.
   */
  public List<VisionMeasurement> drainAllMeasurements() {
    var all = new ArrayList<VisionMeasurement>();
    for (var cam : m_cameras) {
      if (!cam.measurements.isEmpty()) {
        all.addAll(cam.measurements);
        cam.measurements.clear();
      }
    }
    return all;
  }

  @Override
  public void periodic() {
    boolean verbose = Constants.UtilityConstants.kTuningMode;

    for (var cam : m_cameras) {
      if (verbose) {
        m_visionTable.getEntry(cam.name + "/Connected").setBoolean(cam.camera.isConnected());
      }
      processAllResults(cam, verbose);
    }

    // Log critical vision data every 10th cycle (200ms) for match analysis
    if (++m_logCounter >= 10) {
      for (var cam : m_cameras) {
        logVisionMeasurement(cam);
      }
      m_logCounter = 0;
    }
  }

  /**
   * Processes every unread pipeline result for a camera (not just the latest), applies the full
   * filter cascade per frame, and appends accepted measurements to the camera's queue. Also
   * verifies that PhotonVision has returned a calibration matrix — if not, we refuse to compute
   * poses at all (they'd be garbage) and log an edge-triggered warning. After the first successful
   * calibration check, the result is cached and the check is skipped on subsequent cycles.
   */
  private void processAllResults(CameraState cam, boolean verbose) {
    if (!cam.camera.isConnected()) {
      cam.measurements.clear();
      cam.lastAccepted = Optional.empty();
      return;
    }

    // Calibration verification — cached after first success so we don't call
    // getCameraMatrix() every loop once we know intrinsics are loaded.
    if (!cam.calibrationConfirmed) {
      boolean calibrated = cam.camera.getCameraMatrix().isPresent();
      trackCalibrationState(cam, calibrated);
      if (verbose) {
        m_visionTable.getEntry(cam.name + "/Calibrated").setBoolean(calibrated);
      }
      if (!calibrated) {
        return;
      }
      cam.calibrationConfirmed = true;
    }

    var allResults = cam.camera.getAllUnreadResults();
    if (allResults.isEmpty()) {
      return;
    }

    for (var result : allResults) {
      VisionMeasurement measurement = processFrame(result, cam, verbose);
      if (measurement != null) {
        // Cap queue size to prevent unbounded growth if the consumer stalls.
        if (cam.measurements.size() >= kMaxQueuedMeasurements) {
          cam.measurements.remove(0);
        }
        cam.measurements.add(measurement);
        publishMeasurementTelemetry(cam, measurement, verbose);
        cam.lastAccepted = Optional.of(measurement);
      }
    }
  }

  private void trackCalibrationState(CameraState cam, boolean calibrated) {
    if (!calibrated && !cam.uncalibratedLogged) {
      DataLogManager.log("Vision/" + cam.name + " WARNING: camera returned no calibration matrix");
      cam.uncalibratedLogged = true;
    } else if (calibrated && cam.uncalibratedLogged) {
      DataLogManager.log("Vision/" + cam.name + " calibration now available");
      cam.uncalibratedLogged = false;
    }
  }

  private void logVisionMeasurement(CameraState cam) {
    if (!cam.camera.isConnected()) {
      DataLogManager.log("Vision/" + cam.name + " disconnected");
      return;
    }
    if (cam.lastAccepted.isEmpty()) {
      DataLogManager.log("Vision/" + cam.name + " rejected:" + cam.rejectionReason);
      return;
    }
    VisionMeasurement m = cam.lastAccepted.get();
    DataLogManager.log(
        "Vision/"
            + cam.name
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
      CameraState cam, VisionMeasurement measurement, boolean verbose) {
    cam.posePublisher.set(measurement.pose());

    if (verbose) {
      m_visionTable.getEntry(cam.name + "/Accepted").setBoolean(true);
      m_visionTable.getEntry(cam.name + "/RejectionReason").setString("");
      m_visionTable.getEntry(cam.name + "/PoseX_m").setDouble(measurement.pose().getX());
      m_visionTable.getEntry(cam.name + "/PoseY_m").setDouble(measurement.pose().getY());
      m_visionTable
          .getEntry(cam.name + "/PoseHeading_deg")
          .setDouble(measurement.pose().getRotation().getDegrees());
      m_visionTable.getEntry(cam.name + "/StdDevXY").setDouble(measurement.stdDevs().get(0, 0));
      m_visionTable
          .getEntry(cam.name + "/StdDevHeading_deg")
          .setDouble(Math.toDegrees(measurement.stdDevs().get(2, 0)));
    }
  }

  private void publishRejection(CameraState cam, String reason, boolean verbose) {
    cam.rejectionReason = reason;
    cam.rejectionCount++;

    if (verbose) {
      m_visionTable.getEntry(cam.name + "/Accepted").setBoolean(false);
      m_visionTable.getEntry(cam.name + "/RejectionReason").setString(reason);
      m_visionTable.getEntry(cam.name + "/RejectionCount").setDouble(cam.rejectionCount);
    }
  }

  /**
   * Applies the full filter cascade to a single PhotonVision pipeline result. Returns the accepted
   * {@link VisionMeasurement} or {@code null} if the frame was rejected.
   */
  private VisionMeasurement processFrame(
      PhotonPipelineResult result, CameraState cam, boolean verbose) {
    if (!result.hasTargets()) {
      publishRejection(cam, "no_targets", verbose);
      if (verbose) {
        m_visionTable.getEntry(cam.name + "/TagDetected").setBoolean(false);
      }
      return null;
    }

    if (verbose) {
      m_visionTable.getEntry(cam.name + "/TagDetected").setBoolean(true);
      double[] visibleIds = result.targets.stream().mapToDouble(t -> t.getFiducialId()).toArray();
      m_visionTable.getEntry(cam.name + "/VisibleTagIDs").setDoubleArray(visibleIds);
      double[] ambiguities =
          result.targets.stream().mapToDouble(t -> t.getPoseAmbiguity()).toArray();
      m_visionTable.getEntry(cam.name + "/Ambiguities").setDoubleArray(ambiguities);
    }

    int targetCount = result.targets.size();

    if (targetCount < Constants.Vision.kMinAprilTagsForPose) {
      publishRejection(cam, "too_few_targets:" + targetCount, verbose);
      return null;
    }

    if (targetCount == 1
        && result.getBestTarget().getPoseAmbiguity()
            > Constants.Vision.kMaxAcceptableSingleTagAmbiguity) {
      publishRejection(
          cam,
          "ambiguity_too_high:" + String.format("%.2f", result.getBestTarget().getPoseAmbiguity()),
          verbose);
      return null;
    }

    if (targetCount == 1) {
      double distanceToTag =
          result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
      if (distanceToTag > Constants.Vision.kMaxSingleTagDistanceMeters) {
        publishRejection(
            cam, "single_tag_too_far:" + String.format("%.2f", distanceToTag), verbose);
        return null;
      }
    }

    // Attempt coprocessor multi-tag first, fall back to lowest ambiguity
    Optional<EstimatedRobotPose> estimatedPose =
        cam.poseEstimator.estimateCoprocMultiTagPose(result);
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

      estimatedPose = cam.poseEstimator.estimateLowestAmbiguityPose(result);
    }

    if (estimatedPose.isEmpty()) {
      if (verbose) {
        var bestTarget = result.getBestTarget();
        int tagId = bestTarget.getFiducialId();
        boolean tagInLayout = m_FieldLayout.getTagPose(tagId).isPresent();
        double ambiguity = bestTarget.getPoseAmbiguity();
        m_visionTable.getEntry(cam.name + "/ambiguity").setDouble(ambiguity);

        String fallbackReason;
        if (!tagInLayout) {
          fallbackReason = "tag_" + tagId + "_not_in_layout";
        } else if (ambiguity < 0) {
          fallbackReason = "ambiguity_unavailable(tag_" + tagId + ")";
        } else {
          fallbackReason = "lowest_ambiguity_rejected:" + String.format("%.2f", ambiguity);
        }

        publishRejection(
            cam,
            "pose_estimation_failed|multitag:" + multiTagFailReason + "|fallback:" + fallbackReason,
            verbose);
      }
      return null;
    }

    double poseZ = estimatedPose.get().estimatedPose.getZ();
    if (Math.abs(poseZ) > Constants.Vision.kMaxPoseHeightMeters) {
      publishRejection(cam, "z_out_of_range:" + String.format("%.2f", poseZ), verbose);
      return null;
    }

    Pose2d robotPose = estimatedPose.get().estimatedPose.toPose2d();
    double fieldMargin = Constants.Vision.kFieldBoundaryMarginMeters;
    if (robotPose.getX() < -fieldMargin
        || robotPose.getX() > m_FieldLayout.getFieldLength() + fieldMargin
        || robotPose.getY() < -fieldMargin
        || robotPose.getY() > m_FieldLayout.getFieldWidth() + fieldMargin) {
      publishRejection(
          cam,
          "outside_field:" + String.format("(%.2f, %.2f)", robotPose.getX(), robotPose.getY()),
          verbose);
      return null;
    }

    Matrix<N3, N1> stdDevs = computeDynamicStdDevs(estimatedPose.get());

    if (verbose) {
      double[] usedIds =
          estimatedPose.get().targetsUsed.stream().mapToDouble(t -> t.getFiducialId()).toArray();
      m_visionTable.getEntry(cam.name + "/UsedTagIDs").setDoubleArray(usedIds);
    }

    return new VisionMeasurement(
        robotPose, estimatedPose.get().timestampSeconds, stdDevs, cam.name);
  }

  private static Matrix<N3, N1> computeDynamicStdDevs(EstimatedRobotPose estimatedPose) {
    int numTags = estimatedPose.targetsUsed.size();
    boolean isMultiTag = numTags > 1;

    double totalDistance = 0.0;
    for (var target : estimatedPose.targetsUsed) {
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    double avgDistance = totalDistance / numTags;

    double distanceFactor = Math.pow(avgDistance, Constants.Vision.kDistanceExponent);

    double baseXY =
        isMultiTag
            ? Constants.Vision.kMultiTagBaseXYStdDev
            : Constants.Vision.kSingleTagBaseXYStdDev;
    double xyStdDev = baseXY * distanceFactor / numTags;

    // Single-tag heading is unreliable due to the ambiguity problem — ignore it entirely.
    double headingStdDev =
        isMultiTag
            ? Constants.Vision.kMultiTagBaseHeadingStdDev * distanceFactor / numTags
            : Double.POSITIVE_INFINITY;

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
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs, String cameraName) {}
}
