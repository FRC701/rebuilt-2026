// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberExtend;
import frc.robot.commands.ClimberLock;
import frc.robot.commands.ClimberRetract;
import frc.robot.commands.NotShootingCommand;
import frc.robot.commands.PassingCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Agitator.AgitatorState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
  // speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
  // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_DriveField =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain m_DriveTrain = TunerConstants.createDrivetrain();

  private final Agitator m_Agitator = new Agitator();
  private Feeder m_Feeder = new Feeder(FeederConstants.kFeederMotor);
  private Shooter m_LeftShooter = new Shooter(Constants.ShooterConstants.kLeftShooterId);
  private Shooter m_RightShooter = new Shooter(Constants.ShooterConstants.kRightShooterId);
  // Created StartEnd Command for AggitatorToggle
  private Command m_AgitatorToggle =
      Commands.startEnd(
          () -> m_Agitator.m_AgitatorState = AgitatorState.S_On,
          () -> m_Agitator.m_AgitatorState = AgitatorState.S_Off,
          m_Agitator);
  private Command m_FeederToggle =
      Commands.startEnd(
          () -> m_Feeder.m_FeederState = FeederState.S_On,
          () -> m_Feeder.m_FeederState = FeederState.S_Off,
          m_Feeder);

  private final Climber m_Climber;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_Climber = new Climber();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_DriveTrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_DriveTrain.applyRequest(
            () ->
                m_DriveField
                    .withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive
                    // forward
                    // with
                    // negative
                    // Y
                    // (forward)
                    .withVelocityY(
                        -m_driverController.getLeftX()
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_driverController.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with
            // negative X (left)
            ));
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_DriveTrain.applyRequest(() -> idle).ignoringDisable(true));

    m_driverController.a().whileTrue(m_DriveTrain.applyRequest(() -> brake));
    m_driverController
        .b()
        .whileTrue(
            m_DriveTrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y. (FOR TUNING)
    // Note that each routine should be run exactly once in a single log.
    // m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    m_driverController
        .leftTrigger()
        .onTrue(m_DriveTrain.runOnce(() -> m_DriveTrain.seedFieldCentric()));
    m_DriveTrain.registerTelemetry(logger::telemeterize);

    // binds the a-button to toggle the agitator
    m_coDriverController.a().toggleOnTrue(m_AgitatorToggle);
    // Binds the x-button to shooting the shooters
    m_driverController.x().onTrue(new ShootingCommand(m_LeftShooter));
    m_driverController.x().onTrue(new ShootingCommand(m_RightShooter));

    m_driverController.y().onTrue(new PassingCommand(m_LeftShooter));
    m_driverController.y().onTrue(new PassingCommand(m_RightShooter));

    m_driverController.b().onTrue(new NotShootingCommand(m_LeftShooter));
    m_driverController.b().onTrue(new NotShootingCommand(m_RightShooter));
    // pressed,
    // cancelling on release.
    m_driverController.leftBumper().onTrue(new ClimberExtend(m_Climber));
    m_driverController.a().onTrue(new ClimberLock(m_Climber));
    m_driverController.rightBumper().onTrue(new ClimberRetract(m_Climber));
    m_driverController.y().toggleOnTrue(m_FeederToggle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
