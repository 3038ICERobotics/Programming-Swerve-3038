// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
//https://software-metadata.revrobotics.com/REVLib-2025.json
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//   [SWERVE IS IN METERS]   //
// BASE: 0.711m x 0.711m
// WHEELS FROM CENTER: 0.285m
//    [FRONT IS LIGHT]    //
// FLEFT: (0.285, 0.285)
// FRIGHT: (0.285, -0.285)
// BLEFT: (-0.285, 0.285)
// BRIGHT: (-0.285, -0.285)

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Variables used to tune PID - remove once values are defined
  public double Prop, Int, Der, IZone, FeedForward, MinOutput, MaxOutput, MaxRPM;

  // Configuration configurations
  public ClosedLoopConfig VelocityLoopConfig = new ClosedLoopConfig();
  public ClosedLoopConfig SteeringLoopConfig = new ClosedLoopConfig();
  public SparkBaseConfig SteeringBaseConfig = new SparkMaxConfig();
  public SparkBaseConfig VelocityBaseConfig = new SparkMaxConfig();
  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage krackenVelocity = new VelocityVoltage(0).withSlot(0);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  // Constants used to translate RPM to robot speed
  private final int RotationsPerMeter = 27;
  private final int SecondsPerMinute = 60;
  private final double MaxDriveSpeed = .1;
  private final double MaxTurnSpeed = .1;

  public enum ModuleOrder {
    FL, BL, FR, BR
  };

  // Swerve Kinematics
  SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(new Translation2d(0.285, 0.285),
      new Translation2d(-0.285, 0.285),
      new Translation2d(0.285, -0.285), new Translation2d(-0.285, -0.285));

  // Initialize Motors

  SparkMax[] driveMotors = {
      new SparkMax(10, MotorType.kBrushless), // FrontLeftDrive
      new SparkMax(22, MotorType.kBrushless), // BackLeftDrive
      new SparkMax(3, MotorType.kBrushless), // FrontRightDrive
      new SparkMax(14, MotorType.kBrushless),// BackRightDrive
  };

  // TalonFX[] driveMotorsTalon = {
  //     new TalonFX(10),
  //     new TalonFX(22),
  //     new TalonFX(3),
  //     new TalonFX(14),
  // };

  SparkMax[] steerMotors = {
      new SparkMax(15, MotorType.kBrushless), // FrontLeftSteer
      new SparkMax(8, MotorType.kBrushless), // BackLeftSteer
      new SparkMax(1, MotorType.kBrushless), // FrontRightSteer
      new SparkMax(2, MotorType.kBrushless), // BackRightSteer
  };

  // Array
  SparkClosedLoopController[] drivePIDControllers = new SparkClosedLoopController[4];
  SparkClosedLoopController[] steerPIDControllers = new SparkClosedLoopController[4];

  // Initialize Encoders
  RelativeEncoder[] driveEncoders = new RelativeEncoder[4];
  RelativeEncoder[] steerEncoders = new RelativeEncoder[4];

  AnalogContainer[] analogEncoders = new AnalogContainer[4];
  double[] RelativeOffset = { 0, 0, 0, 0 };

  // Initialize Joysticks
  Joystick JoystickL = new Joystick(0);
  Joystick JoystickR = new Joystick(1);

  double BLSTuningSetpoint = 0.0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    TalonFXConfiguration configs = new TalonFXConfiguration();

    /*
     * Voltage-based velocity requires a velocity feed forward to account for the
     * back-emf of the motor
     */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

    for (int i = 0; i < 4; i++) {
      drivePIDControllers[i] = driveMotors[i].getClosedLoopController();
      /* Retry config apply up to 5 times, report if failure */
      // StatusCode status = StatusCode.StatusCodeNotInitialized;
      // for (int j = 0; j < 5; ++j) {
      //   status = driveMotorsTalon[i].getConfigurator().apply(configs);
      //   if (status.isOK())
      //     break;
      // }
      // if (!status.isOK()) {
      //   System.out.println("Could not apply configs, error code: " + status.toString());
      // }
      driveEncoders[i] = driveMotors[i].getEncoder();
      driveEncoders[i].setPosition(0);
      steerPIDControllers[i] = steerMotors[i].getClosedLoopController();
      steerEncoders[i] = steerMotors[i + 4].getEncoder();
      steerEncoders[i].setPosition(0);
      RelativeOffset[i] = 0;
    }
    analogEncoders[0] = new AnalogContainer(steerMotors[0].getAnalog(), 2.23, 1.60);
    analogEncoders[1] = new AnalogContainer(steerMotors[1].getAnalog(), 2.23, 2.18);
    analogEncoders[2] = new AnalogContainer(steerMotors[2].getAnalog(), 2.27, 1.78);
    analogEncoders[3] = new AnalogContainer(steerMotors[3].getAnalog(), 2.20, 0.12);

    VelocityLoopConfig.pidf(0.000170, 0.000001, 0.000020, 0.000001, ClosedLoopSlot.kSlot0);
    VelocityLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);

    SteeringLoopConfig.pidf(1, 0.5, 0.1, 0.00001, ClosedLoopSlot.kSlot0);
    SteeringLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);

    VelocityBaseConfig.apply(VelocityLoopConfig);
    VelocityBaseConfig.inverted(false);

    SteeringBaseConfig.apply(SteeringLoopConfig);
    SteeringBaseConfig.inverted(false);

    for (int i = 0; i < 4; i++) {
      driveMotors[i].configure(VelocityBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      steerMotors[i].configure(SteeringBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // [VARIABLES: 54700 max RPM, ~27 rotations per meter, 60 seconds per minute,
    // ~3.5mps for max speed]

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    AnalogInit();
    PIDTuningInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    PIDTuning();

    double JoystickTolerance = 0.09;
    // Joystick Control
    double TranslateY = -JoystickL.getX() * MaxDriveSpeed;
    double TranslateX = -JoystickL.getY() * MaxDriveSpeed;
    double TranslateRotation = -JoystickR.getX() * MaxTurnSpeed;

    if (Math.abs(TranslateX) < JoystickTolerance)
      TranslateX = 0.0;
    if (Math.abs(TranslateY) < JoystickTolerance)
      TranslateY = 0.0;
    if (Math.abs(TranslateRotation) < JoystickTolerance)
      TranslateRotation = 0.0;

    SwerveModuleState[] OptimizedStates = performKinematicWith(TranslateX, TranslateY, TranslateRotation);

    for (int i = 0; i < 4; i++) {
      drivePIDControllers[i]
          .setReference(OptimizedStates[i].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute,
              SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, FeedForward);
      // driveMotorsTalon[i].setControl(
      //     krackenVelocity.withVelocity(OptimizedStates[i].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute));
      steerPIDControllers[i]
          .setReference(OptimizedStates[i].angle.getRadians() * 55 / (2 * Math.PI) + (RelativeOffset[i] / 360) * 55,
              SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FeedForward);
    }
  }

  public SwerveModuleState[] performKinematicWith(double travelX, double travelY, double rotateRadians) {
    ChassisSpeeds speeds = new ChassisSpeeds(travelX, travelY, rotateRadians);
    SwerveModuleState[] moduleStates = Kinematics.toSwerveModuleStates(speeds);
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = angleMinimize(analogEncoders[i].getDegrees(), moduleStates[i]);
    }
    Kinematics.desaturateWheelSpeeds(moduleStates, MaxDriveSpeed);
    return moduleStates;
  }

  public SwerveModuleState angleMinimize(double CurrentAngle, SwerveModuleState TargetState) {
    double newAngle = TargetState.angle.getDegrees() - CurrentAngle;

    if (newAngle > 90) {
      newAngle -= 180;
      TargetState.speedMetersPerSecond *= -1;
    } else if (newAngle < -90) {
      newAngle += 180;
      TargetState.speedMetersPerSecond *= -1;
    }
    TargetState.angle = new Rotation2d((CurrentAngle + newAngle) * Math.PI / 180);
    return TargetState;
  }

  public void AnalogInit() {
    // initialize the analog offset
    for (int i = 0; i < 4; i++) {
      analogEncoders[i].offset = 0;// analogEncoders[i].maxVolt+analogEncoders[i].sensor.getPosition()-analogEncoders[i].zeroVolt;
      RelativeOffset[i] = analogEncoders[i].offset * 360 / analogEncoders[i].maxVolt;
    }
  }

  public void PIDTuningInit() {
    SmartDashboard.putNumber("PIDSetting: Steer(1) Drive(2) None(0)", 0);
    SmartDashboard.putNumber("P Gain", Prop);
    SmartDashboard.putNumber("I Gain", Int);
    SmartDashboard.putNumber("D Gain", Der);
    SmartDashboard.putNumber("I Zone", IZone);
    SmartDashboard.putNumber("Feed Forward", FeedForward);
    SmartDashboard.putNumber("Max Output", MaxOutput);
    SmartDashboard.putNumber("Min Output", MinOutput);
    SmartDashboard.putNumber("BLSetpoint", 0);
  }

  public void PIDTuning() {
    double tuning = SmartDashboard.getNumber("PIDSetting: Steer(1) Drive(2) None(0)", 0);
    if (tuning != 1 || tuning != 2)
      return;
    double P = SmartDashboard.getNumber("P Gain", 0); // 0.000170
    double I = SmartDashboard.getNumber("I Gain", 0); // 0.000001
    double D = SmartDashboard.getNumber("D Gain", 0); // 0.000020
    double IZ = SmartDashboard.getNumber("I Zone", 0);
    double FF = SmartDashboard.getNumber("Feed Forward", 0); // 0.000001
    double MaxOut = SmartDashboard.getNumber("Max Output", 1);
    double MinOut = SmartDashboard.getNumber("Min Output", -1);

    if (tuning == 1) {
      SteeringLoopConfig.p(P, ClosedLoopSlot.kSlot0);
      SteeringLoopConfig.i(I, ClosedLoopSlot.kSlot0);
      SteeringLoopConfig.d(D, ClosedLoopSlot.kSlot0);
      SteeringLoopConfig.iZone(IZ, ClosedLoopSlot.kSlot0);
      SteeringLoopConfig.velocityFF(FF, ClosedLoopSlot.kSlot0);
      SteeringLoopConfig.outputRange(MaxOut, MinOut, ClosedLoopSlot.kSlot0);
      SteeringBaseConfig.encoder
          .positionConversionFactor(1)
          .velocityConversionFactor(1);
      SteeringBaseConfig.closedLoop
          .pidf(P, I, D, FF, ClosedLoopSlot.kSlot0);
      SteeringLoopConfig.outputRange(-1, 1);
      SteeringBaseConfig.smartCurrentLimit(15); // Default limit is 80A - this limit is too high for a NEO 550
      SteeringBaseConfig.apply(SteeringLoopConfig);
    } else if (tuning == 2) {
      VelocityLoopConfig.p(P, ClosedLoopSlot.kSlot0);
      VelocityLoopConfig.i(I, ClosedLoopSlot.kSlot0);
      VelocityLoopConfig.d(D, ClosedLoopSlot.kSlot0);
      VelocityLoopConfig.iZone(IZ, ClosedLoopSlot.kSlot0);
      VelocityLoopConfig.velocityFF(FF, ClosedLoopSlot.kSlot0);
      VelocityLoopConfig.outputRange(MaxOut, MinOut, ClosedLoopSlot.kSlot0);
      VelocityBaseConfig.encoder
          .positionConversionFactor(1)
          .velocityConversionFactor(1);
      VelocityBaseConfig.closedLoop
          .pidf(P, I, D, FF, ClosedLoopSlot.kSlot0);
      VelocityLoopConfig.outputRange(-1, 1);
      VelocityBaseConfig.smartCurrentLimit(15); // Default limit is 80A - this limit is too high for a NEO 550
      VelocityBaseConfig.apply(VelocityLoopConfig);
    }

    for (int i = 0; i < 4; i++) {
      if (tuning == 2)
        driveMotors[i].configure(VelocityBaseConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
      if (tuning == 1)
        steerMotors[i].configure(SteeringBaseConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }
  }

  public void lowLevelDrive() {

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
