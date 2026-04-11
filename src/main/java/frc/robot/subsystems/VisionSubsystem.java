package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {

  /** Maximum queued measurements per camera before oldest entries are dropped. */
  private static final int kMaxQueuedMeasurements = 20;

  /** All per-camera state in one place — eliminates the 3x field duplication. */
  private static class CameraState {
    final String name; // NT table name for LimelightHelpers calls
    final String displayName; // for logging and telemetry
    final StructPublisher<Pose2d> posePublisher;
    final List<VisionMeasurement> measurements = new ArrayList<>();
    Optional<VisionMeasurement> lastAccepted = Optional.empty();
    int rejectionCount = 0;
    String rejectionReason = "";

    CameraState(String name, String displayName) {
      this.name = name;
      this.displayName = displayName;
      this.posePublisher =
          NetworkTableInstance.getDefault()
              .getStructTopic("Vision/" + displayName + "/Pose", Pose2d.struct)
              .publish();
    }
  }

  private final AprilTagFieldLayout m_FieldLayout;
  private final CameraState[] m_cameras;

  private final NetworkTable m_visionTable = NetworkTableInstance.getDefault().getTable("Vision");

  // Gyro heading supplier — injected by RobotContainer after drivetrain is constructed.
  // Used to feed MegaTag2 orientation. Defaults to zero heading until wired.
  private Supplier<Rotation2d> m_gyroSupplier = Rotation2d::new;

  // Logging — throttle to every 10th cycle (200ms)
  private int m_logCounter = 0;

  public VisionSubsystem() {
    m_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    m_cameras =
        new CameraState[] {
          new CameraState(Constants.Vision.kRightCameraName, "Right"),
          new CameraState(Constants.Vision.kForwardCameraName, "Forward"),
          new CameraState(Constants.Vision.kReverseCameraName, "Reverse"),
        };
  }

  /**
   * Injects the gyro heading supplier used for MegaTag2 orientation. Must be called during robot
   * construction (typically from {@code RobotContainer}) before the scheduler starts running {@link
   * #periodic()}.
   */
  public void setGyroHeadingSupplier(Supplier<Rotation2d> supplier) {
    m_gyroSupplier = supplier;
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

    // Feed gyro heading to all cameras for MegaTag2 orientation-constrained solving.
    double yawDeg = m_gyroSupplier.get().getDegrees();
    for (var cam : m_cameras) {
      LimelightHelpers.SetRobotOrientation(cam.name, yawDeg, 0, 0, 0, 0, 0);
    }

    for (var cam : m_cameras) {
      if (verbose) {
        m_visionTable
            .getEntry(cam.displayName + "/Connected")
            .setBoolean(LimelightHelpers.getTV(cam.name) || hasRecentHeartbeat(cam.name));
      }
      processCamera(cam, verbose);
    }

    // Log critical vision data every 10th cycle (200ms) for match analysis
    if (++m_logCounter >= 10) {
      for (var cam : m_cameras) {
        logVisionMeasurement(cam);
      }
      m_logCounter = 0;
    }
  }

  /** Returns true if the Limelight heartbeat has incremented recently. */
  private boolean hasRecentHeartbeat(String cameraName) {
    return LimelightHelpers.getHeartbeat(cameraName) > 0;
  }

  /**
   * Gets a pose estimate from a single Limelight camera, applies the filter cascade, and queues
   * accepted measurements. Tries MegaTag2 first, falls back to MegaTag1.
   */
  private void processCamera(CameraState cam, boolean verbose) {
    // Try MegaTag2 first (orientation-constrained, more accurate)
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam.name);
    boolean isMegaTag2 = true;

    if (!LimelightHelpers.validPoseEstimate(estimate) || estimate.tagCount == 0) {
      // Fall back to MegaTag1
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cam.name);
      isMegaTag2 = false;

      if (!LimelightHelpers.validPoseEstimate(estimate) || estimate.tagCount == 0) {
        publishRejection(cam, "no_valid_estimate", verbose);
        cam.lastAccepted = Optional.empty();
        return;
      }
    }

    VisionMeasurement measurement = processEstimate(estimate, cam, isMegaTag2, verbose);
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

  /**
   * Applies the full filter cascade to a Limelight PoseEstimate. Returns the accepted {@link
   * VisionMeasurement} or {@code null} if the estimate was rejected.
   */
  private VisionMeasurement processEstimate(
      PoseEstimate estimate, CameraState cam, boolean isMegaTag2, boolean verbose) {
    int tagCount = estimate.tagCount;

    if (verbose) {
      m_visionTable.getEntry(cam.displayName + "/TagDetected").setBoolean(true);
      m_visionTable.getEntry(cam.displayName + "/TagCount").setDouble(tagCount);
      m_visionTable.getEntry(cam.displayName + "/AvgTagDist").setDouble(estimate.avgTagDist);
      m_visionTable.getEntry(cam.displayName + "/MegaTag2").setBoolean(isMegaTag2);

      if (estimate.rawFiducials != null) {
        double[] visibleIds = new double[estimate.rawFiducials.length];
        double[] ambiguities = new double[estimate.rawFiducials.length];
        for (int i = 0; i < estimate.rawFiducials.length; i++) {
          visibleIds[i] = estimate.rawFiducials[i].id;
          ambiguities[i] = estimate.rawFiducials[i].ambiguity;
        }
        m_visionTable.getEntry(cam.displayName + "/VisibleTagIDs").setDoubleArray(visibleIds);
        m_visionTable.getEntry(cam.displayName + "/Ambiguities").setDoubleArray(ambiguities);
      }
    }

    if (tagCount < Constants.Vision.kMinAprilTagsForPose) {
      publishRejection(cam, "too_few_targets:" + tagCount, verbose);
      return null;
    }

    // Single-tag ambiguity check
    if (tagCount == 1
        && estimate.rawFiducials != null
        && estimate.rawFiducials.length > 0
        && estimate.rawFiducials[0].ambiguity > Constants.Vision.kMaxAcceptableSingleTagAmbiguity) {
      publishRejection(
          cam,
          "ambiguity_too_high:" + String.format("%.2f", estimate.rawFiducials[0].ambiguity),
          verbose);
      return null;
    }

    // Single-tag distance check
    if (tagCount == 1 && estimate.avgTagDist > Constants.Vision.kMaxSingleTagDistanceMeters) {
      publishRejection(
          cam, "single_tag_too_far:" + String.format("%.2f", estimate.avgTagDist), verbose);
      return null;
    }

    Pose2d robotPose = estimate.pose;

    // Field boundary check
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

    Matrix<N3, N1> stdDevs = computeDynamicStdDevs(tagCount, estimate.avgTagDist);

    if (verbose) {
      if (estimate.rawFiducials != null) {
        double[] usedIds = new double[estimate.rawFiducials.length];
        for (int i = 0; i < estimate.rawFiducials.length; i++) {
          usedIds[i] = estimate.rawFiducials[i].id;
        }
        m_visionTable.getEntry(cam.displayName + "/UsedTagIDs").setDoubleArray(usedIds);
      }
    }

    return new VisionMeasurement(robotPose, estimate.timestampSeconds, stdDevs, cam.displayName);
  }

  private void logVisionMeasurement(CameraState cam) {
    if (cam.lastAccepted.isEmpty()) {
      DataLogManager.log("Vision/" + cam.displayName + " rejected:" + cam.rejectionReason);
      return;
    }
    VisionMeasurement m = cam.lastAccepted.get();
    DataLogManager.log(
        "Vision/"
            + cam.displayName
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
      m_visionTable.getEntry(cam.displayName + "/Accepted").setBoolean(true);
      m_visionTable.getEntry(cam.displayName + "/RejectionReason").setString("");
      m_visionTable.getEntry(cam.displayName + "/PoseX_m").setDouble(measurement.pose().getX());
      m_visionTable.getEntry(cam.displayName + "/PoseY_m").setDouble(measurement.pose().getY());
      m_visionTable
          .getEntry(cam.displayName + "/PoseHeading_deg")
          .setDouble(measurement.pose().getRotation().getDegrees());
      m_visionTable
          .getEntry(cam.displayName + "/StdDevXY")
          .setDouble(measurement.stdDevs().get(0, 0));
      m_visionTable
          .getEntry(cam.displayName + "/StdDevHeading_deg")
          .setDouble(Math.toDegrees(measurement.stdDevs().get(2, 0)));
    }
  }

  private void publishRejection(CameraState cam, String reason, boolean verbose) {
    cam.rejectionReason = reason;
    cam.rejectionCount++;

    if (verbose) {
      m_visionTable.getEntry(cam.displayName + "/Accepted").setBoolean(false);
      m_visionTable.getEntry(cam.displayName + "/RejectionReason").setString(reason);
      m_visionTable.getEntry(cam.displayName + "/RejectionCount").setDouble(cam.rejectionCount);
    }
  }

  private static Matrix<N3, N1> computeDynamicStdDevs(int tagCount, double avgDistance) {
    boolean isMultiTag = tagCount > 1;

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

  /** Stub for simulation — Limelight has no sim equivalent. */
  public void setSimRobotPose(Pose2d pose) {
    // No-op: Limelight does not support desktop simulation.
  }

  @Override
  public void simulationPeriodic() {
    // No-op: Limelight does not support desktop simulation.
  }

  public static record VisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs, String cameraName) {}
}
