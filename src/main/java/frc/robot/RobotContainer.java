// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.FeederOn;
import frc.robot.commands.NotShootingCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> autoChooser;
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

  // Instantiating the Subsystems

  public final CommandSwerveDrivetrain m_DriveTrain = TunerConstants.createDrivetrain();
  private final Agitator m_Agitator = new Agitator();
  private Intake m_Intake = new Intake(m_Agitator);
  private Feeder m_LeftFeeder = new Feeder(FeederConstants.kFeederLeftMotor, "Left Feeder");
  private Feeder m_RightFeeder = new Feeder(FeederConstants.kFeederRightMotor, "Right Feeder");
  private Shooter m_LeftShooter =
      new Shooter(Constants.ShooterConstants.kLeftShooterId, "Left Shooter", m_Agitator);
  private Shooter m_RightShooter =
      new Shooter(Constants.ShooterConstants.kRightShooterId, "Right Shooter", m_Agitator);
  private Climber m_Climber = new Climber();

  // Instantiating Shooter Commands

  private ShootingCommand m_ShootingCommand = new ShootingCommand(m_LeftShooter, m_RightShooter);
  private ShootCommand m_ShootCommand = new ShootCommand(m_LeftShooter, m_RightShooter);
  private FeederOn m_FeederOn = new FeederOn(m_LeftFeeder, m_RightFeeder);
  private NotShootingCommand m_NotShootingCommand =
      new NotShootingCommand(m_LeftShooter, m_RightShooter, m_LeftFeeder, m_RightFeeder);

  // Instantiating the Toggles

  private Command m_IntakeToggle =
      Commands.startEnd(
          () -> m_Intake.m_IntakeState = IntakeState.S_Extend,
          () -> m_Intake.m_IntakeState = IntakeState.S_Retract,
          m_Intake,
          m_Agitator);

  private Command m_ShooterToggle = Commands.startEnd(() -> m_ShootingCommand.andThen(m_ShootCommand.andThen(m_FeederOn)), () -> m_NotShootingCommand.initialize(), m_LeftShooter, m_RightShooter, m_LeftFeeder, m_RightFeeder);

  //   private Command m_IntakeRollerToggle =
  //       Commands.startEnd(() -> m_Intake.holdBool = true, () -> m_Intake.holdBool = false,
  //    m_Intake);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_xboxController =
      new CommandXboxController(OperatorConstants.kXboxControllerPort);
  private final CommandPS4Controller m_ps4Controller =
      new CommandPS4Controller(OperatorConstants.kPs4ControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // NamedCommands.registerCommand(
    //    "ShootingCommand", new ShootingCommand(m_LeftShooter, m_RightShooter));
    NamedCommands.registerCommand("ShootCommand", new ShootCommand(m_LeftShooter, m_RightShooter));
    NamedCommands.registerCommand("FeederOn", new FeederOn(m_LeftFeeder, m_RightFeeder));
    NamedCommands.registerCommand("ExtendIntake", new ExtendIntake(m_Intake));
    // builds auto chooser
    autoChooser = AutoBuilder.buildAutoChooser("Shoot");
    SmartDashboard.putData("Auto Mode", autoChooser);
    // Configure the trigger bindings
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
                    .withVelocityX(
                        -m_ps4Controller.getLeftY()
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -m_ps4Controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_ps4Controller.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with
            // negative X (left)
            ));
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_DriveTrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y. (FOR TUNING)
    // Note that each routine should be run exactly once in a single log.
    // m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Driver Bindings
    // Reset the field-centric heading on left bumper press for Xbox
    m_xboxController
        .leftBumper()
        .onTrue(m_DriveTrain.runOnce(() -> m_DriveTrain.seedFieldCentric()));
    // Playstation variant of ^^^
    m_ps4Controller.L1().onTrue(m_DriveTrain.runOnce(() -> m_DriveTrain.seedFieldCentric()));
    m_DriveTrain.registerTelemetry(logger::telemeterize);

    // Climber Bindings
    // m_driverController
    //     .a()
    //     .onTrue(
    //         new ClimberUpDownToggle(
    //             m_Climber, m_Agitator, m_Feeder, m_LeftShooter, m_RightShooter));
    // // Playstation variant of ^^^
    // m_ps4Controller
    //     .cross()
    //     .onTrue(
    //         new ClimberUpDownToggle(
    //             m_Climber, m_Agitator, m_Feeder, m_LeftShooter, m_RightShooter));

    // m_driverController.b().onTrue(new ClimberLock(m_Climber));
    // // Playstation variant of ^^^
    // m_ps4Controller.circle().onTrue(new ClimberLock(m_Climber));

    // Intake Bindings
    m_xboxController.leftTrigger().toggleOnTrue(m_IntakeToggle);
    // Playstation variant of ^^^
    m_ps4Controller.L2().toggleOnTrue(m_IntakeToggle);
    // m_ps4Controller.povRight().onTrue(m_IntakeRollerToggle);

    // Shooter Binding XBox
    m_xboxController.rightTrigger().onTrue(m_ShooterToggle);
    // Playstation variant of ^^^
    m_ps4Controller.R2().onTrue(m_ShooterToggle);

    m_ps4Controller
        .povUp()
        .whileTrue(
            m_DriveTrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(1, 0))));

    // Agitator Bindings
    // binds the dpad down to toggle the agitator for Xbox
    // m_xboxController.povDown().toggleOnTrue(m_AgitatorToggle);
    // // Playstation variant of ^^^
    // m_ps4Controller.povDown().toggleOnTrue(m_AgitatorToggle);
  }

  public Command getAutonomusCommand() {
    // An exmaple command will run in automonous
    // Run the path selected form the auto chooser
    return autoChooser.getSelected();
  }
}
