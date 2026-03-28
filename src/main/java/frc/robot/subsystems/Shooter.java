// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Agitator.AgitatorState;

public class Shooter extends SubsystemBase {

  private TalonFX m_ShooterMotor;

  public ShooterEnumState m_ShooterEnumState;

  private Agitator m_Agitator;
  private CommandSwerveDrivetrain m_drivetrain;
  int aState;

  // Boolean to track the enabled status
  private boolean m_ShooterEnabled = true;

  // voltSpeed = desired amount of rotations per second
  private VelocityVoltage voltSpeed = new VelocityVoltage(0).withSlot(0);

  private StatusSignal<AngularVelocity> velocitySignal;

  private String m_StateString;
  private String m_EnabledString;
  private String m_RevolutionsErrorString;
  private String m_SpeedString;

  // Simulation
  private FlywheelSim m_flywheelSim;
  private TalonFXSimState m_motorSimState;

  private final double[] distance = {
    Units.inchesToMeters(36), Units.inchesToMeters(61), Units.inchesToMeters(72)
  };
  private final double[] speed = {50, 75, 100};
  private double a, b, c;

  /** Creates a new Shooter. */
  public Shooter(
      int motorId, String subsystemName, CommandSwerveDrivetrain drivetrain, Agitator agitator) {
    super(subsystemName);

    // gives values to the Strings that are used for Shuffleboard
    nameStrings();
    m_Agitator = agitator;
    m_drivetrain = drivetrain;

    // Selects the intial state
    m_ShooterEnumState = ShooterEnumState.S_NotShooting;

    // Identifying of the motor
    m_ShooterMotor = new TalonFX(motorId);

    // Creates the Configs
    var m_TalonFXConfig =
        new TalonFXConfiguration()
            .withVoltage(
                new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10));

    MotorOutputConfigs shooterConfig = m_TalonFXConfig.MotorOutput;

    if (motorId == Constants.ShooterConstants.kRightShooterId) {
      shooterConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      shooterConfig.Inverted = InvertedValue.Clockwise_Positive;
    }

    // Configs that use the PID values to help with motor speed
    Slot0Configs Slot0Configs = m_TalonFXConfig.Slot0;
    Slot0Configs.kP = Constants.ShooterConstants.kP;
    Slot0Configs.kI = Constants.ShooterConstants.kI;
    Slot0Configs.kD = Constants.ShooterConstants.kD;
    Slot0Configs.kV = Constants.ShooterConstants.kV;
    Slot0Configs.kS = Constants.ShooterConstants.kS;

    // Applying the configs to the motors, PID
    // Applying the configs to the motors, Voltage Limits
    m_ShooterMotor.getConfigurator().apply(m_TalonFXConfig);

    velocitySignal = m_ShooterMotor.getVelocity();

    BallShooterInterpolation(distance[0], speed[0], distance[1], speed[1], distance[2], speed[2]);

    if (Utils.isSimulation()) {
      m_motorSimState = m_ShooterMotor.getSimState();
      m_flywheelSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  DCMotor.getKrakenX60Foc(1),
                  Constants.ShooterConstants.kSimMOI,
                  Constants.ShooterConstants.kSimGearRatio),
              DCMotor.getKrakenX60Foc(1));
    }
  }

  private void nameStrings() {
    m_StateString = getName() + " State";
    m_EnabledString = getName() + " Enabled";
    m_RevolutionsErrorString = getName() + "Revolutions Error";
    m_SpeedString = getName() + " Speed";
  }

  public double VoltageCheck() {
    return m_ShooterMotor.getVelocity().getValueAsDouble();
  }

  // returns true if no balls in shooter
  public boolean CurrentCHeck() {
    return m_ShooterMotor.getStatorCurrent().getValueAsDouble() <= 30; // placeholder
  }

  public boolean UpToSpeed() {
    return (VoltageCheck() <= Constants.ShooterConstants.shootRev + 3
        && VoltageCheck() >= Constants.ShooterConstants.shootRev - 3);
  }

  public enum ShooterEnumState {
    S_Shooting,
    S_Passing,
    S_NotShooting
  }

  public void runShooterStates() {
    switch (m_ShooterEnumState) {
      case S_Shooting:
        shooting();
        break;
      case S_Passing:
        passing();
        break;
      case S_NotShooting:
        stopping();
        break;
    }
  }

  // Uses PID to arrive at our shooting speed
  public void shooting() {
    m_ShooterMotor.setControl(voltSpeed.withVelocity(Constants.ShooterConstants.shootRev));
    m_Agitator.m_AgitatorState = AgitatorState.S_In;
  }

  // Uses PID to arrive at our passing speed
  public void passing() {
    m_ShooterMotor.setControl(voltSpeed.withVelocity(Constants.ShooterConstants.passRev));
    m_Agitator.m_AgitatorState = AgitatorState.S_In;
  }

  // Sets the speed to 0 by using a VelocityVotage object with 0 velocity
  public void stopping() {
    m_ShooterMotor.setControl(voltSpeed.withVelocity(0));
    if (aState == 0) m_Agitator.m_AgitatorState = AgitatorState.S_Off;
    else if (aState == 1) m_Agitator.m_AgitatorState = AgitatorState.S_Idle;
  }

  private boolean setEnabledStatus(boolean shooterStatus) {
    m_ShooterEnabled = shooterStatus;
    return m_ShooterEnabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // The current State
    // SmartDashboard.putString(m_StateString, m_ShooterEnumState.toString());
    // SmartDashboard.putBoolean(m_EnabledString, m_ShooterEnabled);
    // SmartDashboard.putNumber(
    //     "RevolutionsError", m_ShooterMotor.getClosedLoopError().refresh().getValueAsDouble());
    // SmartDashboard.putNumber("ShooterSpeed", velocitySignal.getValueAsDouble());
    // SmartDashboard.putNumber(
    //     "ShooterCurrent", m_ShooterMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putBoolean("NoBallsInShooter", CurrentCHeck());
    // SmartDashboard.putBoolean("ShooterUpToSpeed", UpToSpeed());
    velocitySignal.refresh();

    setEnabledStatus(SmartDashboard.getBoolean(m_EnabledString, m_ShooterEnabled));
    if (m_ShooterEnabled) {
      runShooterStates();
    }

    if (m_Agitator.m_AgitatorState == AgitatorState.S_Off) aState = 0;
    else if (m_Agitator.m_AgitatorState == AgitatorState.S_Idle) aState = 1;
  }

  private static final AprilTagFieldLayout kFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private static final Translation2d kRedHubPosition =
      new Translation2d(
          Constants.AimBotConstants.kRedHubXMeters, Constants.AimBotConstants.kRedHubYMeters);

  private static final Translation2d kBlueHubPosition =
      new Translation2d(
          kFieldLayout.getFieldLength() - Constants.AimBotConstants.kRedHubXMeters,
          kFieldLayout.getFieldWidth() - Constants.AimBotConstants.kRedHubYMeters);

  double DistanceToHub() {
    Pose2d currentPose = m_drivetrain.getState().Pose;
    // TODO: Can isRed and hubPosition be calculated once at startup?
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    Translation2d hubPosition = isRed ? kRedHubPosition : kBlueHubPosition;
    Translation2d robotToHub = hubPosition.minus(currentPose.getTranslation());
    double DistanceToHub_m = robotToHub.getNorm();
    return DistanceToHub_m;
  }

  // Call once at startup — pre-computes the quadratic coefficients
  void BallShooterInterpolation(double x0, double y0, double x1, double y1, double x2, double y2) {
    // Lagrange basis pre-expanded into ax^2 + bx + c
    double d0 = (x0 - x1) * (x0 - x2);
    double d1 = (x1 - x0) * (x1 - x2);
    double d2 = (x2 - x0) * (x2 - x1);

    a = y0 / d0 + y1 / d1 + y2 / d2;
    b = y0 * (-x1 - x2) / d0 + y1 * (-x0 - x2) / d1 + y2 * (-x0 - x1) / d2;
    c = y0 * (x1 * x2) / d0 + y1 * (x0 * x2) / d1 + y2 * (x0 * x1) / d2;
  }

  // Call in the robot loop — just 2 multiplies + 2 adds (Horner's method)
  double speed(double distance) {
    return (a * distance + b) * distance + c;
  }

  @Override
  public void simulationPeriodic() {
    if (m_motorSimState == null) return;

    m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_flywheelSim.setInputVoltage(m_motorSimState.getMotorVoltage());
    m_flywheelSim.update(0.02);

    double mechanismRadPerSec = m_flywheelSim.getAngularVelocityRadPerSec();
    double rotorRPS =
        mechanismRadPerSec / (2.0 * Math.PI) * Constants.ShooterConstants.kSimGearRatio;

    m_motorSimState.addRotorPosition(rotorRPS * 0.02);
    m_motorSimState.setRotorVelocity(rotorRPS);
  }
}
