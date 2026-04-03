package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility.LoggedTunableNumber;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class AimAtHub extends Command {
  private static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private static final AprilTagFieldLayout kFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private static final Translation2d kRedHubPosition =
      new Translation2d(
          Constants.AimBotConstants.kRedHubXMeters, Constants.AimBotConstants.kRedHubYMeters);

  private static final Translation2d kBlueHubPosition =
      new Translation2d(
          kFieldLayout.getFieldLength() - Constants.AimBotConstants.kRedHubXMeters,
          kFieldLayout.getFieldWidth() - Constants.AimBotConstants.kRedHubYMeters);

  private final CommandSwerveDrivetrain m_drivetrain;
  private final DoubleSupplier m_velocityXSupplier;
  private final DoubleSupplier m_velocityYSupplier;
  private final SwerveRequest.FieldCentricFacingAngle m_request;

  /**
   * Creates a command that aims the robot at the hub while allowing the driver to control
   * translation. The robot will automatically rotate to face the hub target.
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param velocityX Supplier for forward velocity (field-relative, m/s)
   * @param velocityY Supplier for left velocity (field-relative, m/s)
   */
  public AimAtHub(
      CommandSwerveDrivetrain drivetrain, DoubleSupplier velocityX, DoubleSupplier velocityY) {
    m_drivetrain = drivetrain;
    m_velocityXSupplier = velocityX;
    m_velocityYSupplier = velocityY;

    m_request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(kMaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    m_request.HeadingController.setPID(
        Constants.AimBotConstants.kP.get(),
        Constants.AimBotConstants.kI.get(),
        Constants.AimBotConstants.kD.get());
    m_request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            m_request.HeadingController.setPID(
                Constants.AimBotConstants.kP.get(),
                Constants.AimBotConstants.kI.get(),
                Constants.AimBotConstants.kD.get()),
        Constants.AimBotConstants.kP,
        Constants.AimBotConstants.kI,
        Constants.AimBotConstants.kD);

    Pose2d currentPose = m_drivetrain.getState().Pose;
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Translation2d hubPosition = isRed ? kRedHubPosition : kBlueHubPosition;
    Translation2d robotToHub = hubPosition.minus(currentPose.getTranslation());
    // Point the front of the robot (where the shooter is) toward the hub
    Rotation2d targetAngle = robotToHub.getAngle();

    // Compensate for operator perspective (Red alliance uses 180° perspective)
    // FieldCentricFacingAngle interprets angles relative to operator perspective,
    // so we must subtract the perspective to maintain field-absolute aiming
    Rotation2d operatorPerspective = isRed ? Rotation2d.k180deg : Rotation2d.kZero;
    targetAngle = targetAngle.minus(operatorPerspective);

    SmartDashboard.putNumber("AimBot/TargetAngle_deg", targetAngle.getDegrees());
    SmartDashboard.putNumber("AimBot/DistanceToHub_m", robotToHub.getNorm());
    SmartDashboard.putNumber("currentPose", currentPose.getRotation().getDegrees());

    m_drivetrain.setControl(
        m_request
            .withVelocityX(m_velocityXSupplier.getAsDouble())
            .withVelocityY(m_velocityYSupplier.getAsDouble())
            .withTargetDirection(targetAngle));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
