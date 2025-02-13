// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
//https://software-metadata.revrobotics.com/REVLib-2025.json

/***************************************************************************************
 *                              IMPORTS                                                *
***************************************************************************************/
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

/***************************************************************************************
 *                              ROBOT CLASS                                            *
 *                                                                                     *
 * The methods in this class are called automatically corresponding to each mode,      *
 * as described in the TimedRobot documentation. If you change the name of this class  *
 * or the package after creating this project, you must also update the Main.java      *
 * file in the project.                                                                *
***************************************************************************************/
public class Robot extends TimedRobot {
  /* Auto Selection */
  private static final String kDefaultAuto  = "Default";
  private static final String kCustomAuto   = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /* Variables used to tune PID - remove once values are defined - TODO */
  public double Prop, Int, Der, IZone, FeedForward, MinOutput, MaxOutput, MaxRPM;

  /* Spark Max Configurations */
  public ClosedLoopConfig VelocityLoopConfig  = new ClosedLoopConfig();
  public ClosedLoopConfig SteeringLoopConfig  = new ClosedLoopConfig();
  public SparkBaseConfig SteeringBaseConfig   = new SparkMaxConfig();
  public SparkBaseConfig VelocityBaseConfig   = new SparkMaxConfig();

  /* Constants used to translate RPM to robot speed */
  private final int     RotationsPerMeter = 27;
  private final int     SecondsPerMinute  = 60;
  private final double  MaxDriveSpeed     = .1;
  private final double  MaxTurnSpeed      = .1;

  public enum ModuleOrder {
    FL, BL, FR, BR, SizeOfModuleOrder
  };

  /* Swerve Kinematics Setup */
  SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(new Translation2d(0.285, 0.285), // FL
                                                               new Translation2d(-0.285, 0.285),  // BL
                                                               new Translation2d(0.285, -0.285),  // FR 
                                                               new Translation2d(-0.285, -0.285));  // BR

  /* Initialize Drive Motors */
  // TODO - define device IDs as constants in another file
  SparkMax[] driveMotors = {
      new SparkMax(10, MotorType.kBrushless),   // FrontLeftDrive
      new SparkMax(22, MotorType.kBrushless),   // BackLeftDrive
      new SparkMax(3,  MotorType.kBrushless),   // FrontRightDrive
      new SparkMax(14, MotorType.kBrushless),   // BackRightDrive
  };

  /* Initialize Steer Motors */ 
  // TODO - define device IDs as constants in another file
  SparkMax[] steerMotors = {
      new SparkMax(15, MotorType.kBrushless),   // FrontLeftSteer
      new SparkMax(8, MotorType.kBrushless),    // BackLeftSteer
      new SparkMax(1, MotorType.kBrushless),    // FrontRightSteer
      new SparkMax(2, MotorType.kBrushless),    // BackRightSteer
  };

  /* Initialize Array of Controllers for Each Motor Type */
  SparkClosedLoopController[] drivePIDControllers = new SparkClosedLoopController[ModuleOrder.SizeOfModuleOrder.ordinal()];
  SparkClosedLoopController[] steerPIDControllers = new SparkClosedLoopController[ModuleOrder.SizeOfModuleOrder.ordinal()];

  /* Initialize Encoders */
  // TODO - get rid of magic number '4' like above
  RelativeEncoder[] driveEncoders   = new RelativeEncoder[4];
  RelativeEncoder[] steerEncoders   = new RelativeEncoder[4];
  AnalogContainer[] analogEncoders  = new AnalogContainer[4];

  double[] AnalogEncoderDegrees = {0.0, 0.0f, 0.0f, 0.0f};

  double[] RelativeOffset = { 0, 0, 0, 0 }; // TODO

  /* Initialize Joysticks */
  Joystick JoystickL = new Joystick(0);
  Joystick JoystickR = new Joystick(1);

  /***************************************************************************************
   *                              ROBOT TASK                                             *
   *                                                                                     *
   * This function is run when the robot is first started up and should be used          *
   * for any initialization code.                                                        *
  ***************************************************************************************/
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /* Set the Motor Controller Configurations */
    for (int i = 0; i < ModuleOrder.SizeOfModuleOrder.ordinal(); i++) {

      /* Populate the motor controllers and encoders */
      steerPIDControllers[i] = steerMotors[i].getClosedLoopController();
      drivePIDControllers[i] = driveMotors[i].getClosedLoopController();
      steerEncoders[i] = steerMotors[i].getEncoder();
      driveEncoders[i] = driveMotors[i].getEncoder();
    
       /* reset encoders and offsets to 0 */
      driveEncoders[i].setPosition(0);
      steerEncoders[i].setPosition(0);
      RelativeOffset[i] = 0;
    }

    /* Create new Analog Encoder Objects and update analog Encoder Array */
    analogEncoders[ModuleOrder.FL.ordinal()] = new AnalogContainer(steerMotors[ModuleOrder.FL.ordinal()].getAnalog(), 2.23, 1.60);
    analogEncoders[ModuleOrder.BL.ordinal()] = new AnalogContainer(steerMotors[ModuleOrder.BL.ordinal()].getAnalog(), 2.23, 2.18);
    analogEncoders[ModuleOrder.FR.ordinal()] = new AnalogContainer(steerMotors[ModuleOrder.FR.ordinal()].getAnalog(), 2.27, 1.78);
    analogEncoders[ModuleOrder.BR.ordinal()] = new AnalogContainer(steerMotors[ModuleOrder.BR.ordinal()].getAnalog(), 2.20, 0.12);

    /* Set velocity loop control parameters */
    // TODO - create constants for the control gains
    VelocityLoopConfig.pidf(0.000170, 0.000001, 0.000020, 0.000001, ClosedLoopSlot.kSlot0);
    VelocityLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);
    VelocityBaseConfig.apply(VelocityLoopConfig);
    VelocityBaseConfig.inverted(false);

    /* Set steering loop control parameters */
    // TODO - create constants for the control gains
    SteeringBaseConfig.smartCurrentLimit(15); // Default limit is 80A - this limit is too high for a NEO 550
    SteeringLoopConfig.pidf(1, 0.5, 0.1, 0.00001, ClosedLoopSlot.kSlot0);
    SteeringLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);
    SteeringBaseConfig.apply(SteeringLoopConfig);
    SteeringBaseConfig.inverted(false);

    /* apply the steering and velocity configurations to the motor controller object */
    for (int i = 0; i < ModuleOrder.SizeOfModuleOrder.ordinal(); i++) {
      driveMotors[i].configure(VelocityBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      steerMotors[i].configure(SteeringBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /***************************************************************************************/
  /*                           ROBOT PERIODIC TASK                                       */
  /***************************************************************************************/
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

  /***************************************************************************************/
  /*                           AUTONOMOUS INIT                                           */
  /***************************************************************************************/
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
  // TODO - Add a header here similar to above
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
  }

  /***************************************************************************************/
  /*                           AUTONOMOUS PERIODIC                                       */
  /***************************************************************************************/
  /** This function is called periodically during autonomous. */
  @Override
  // TODO - Add a header here similar to above
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // TODO Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // TODO Put default auto code here
        break;
    }
  }

  /***************************************************************************************/
  /*                           TELEOP INIT                                               */
  /***************************************************************************************/
  /** This function is called once when teleop is enabled. */
  @Override
  // TODO - Add a header here similar to above
  public void teleopInit() {
    AnalogInit();
    PIDTuningInit();
  }

  /***************************************************************************************/
  /*                           TELEOP PERIODIC                                           */
  /***************************************************************************************/
  /** This function is called periodically during operator control. */
  @Override
  // TODO - Add a header here similar to above
  public void teleopPeriodic() {

    /* Initialize Joystick Parameters */
    double JoystickTolerance = 0.09; // TODO move to a top level define/constant
    double TranslateY = -JoystickL.getX() * MaxDriveSpeed;
    double TranslateX = -JoystickL.getY() * MaxDriveSpeed;
    double TranslateRotation = -JoystickR.getX() * MaxTurnSpeed;

    /* Apply Joystick Tolerance */
    if (Math.abs(TranslateX) < JoystickTolerance)
      TranslateX = 0.0;
    if (Math.abs(TranslateY) < JoystickTolerance)
      TranslateY = 0.0;
    if (Math.abs(TranslateRotation) < JoystickTolerance)
      TranslateRotation = 0.0;

    /* Get Speed and Angle commands for the swerve modules */
    SwerveModuleState[] OptimizedStates = performKinematicWith(TranslateX, TranslateY, TranslateRotation);

    /* For every swerve module */
    for (int i = 0; i < ModuleOrder.SizeOfModuleOrder.ordinal(); i++) {
      /* set the speed target */
      drivePIDControllers[i]
          .setReference(OptimizedStates[i].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute,
              SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, FeedForward);
      /* set the angle target */
      
      // This logic should make it so we never have to reset the wheels to 0
      // Math to get to new setpoint
      // delta between actual and commanded angle
      double angleDelta = (OptimizedStates[i].angle.getRadians() * 360/(2*Math.PI)) - analogEncoders[i].getDegrees();
      // translate angle delta to rotations
      double rotationsDelta = (angleDelta * 55/360.0);
      // Look at current position in rotations, add delta for new setpoint
      double newSetpoint = rotationsDelta + steerEncoders[i].getPosition();
      steerPIDControllers[i]
          .setReference(-newSetpoint, //OptimizedStates[i].angle.getRadians() * 55 / (2 * Math.PI) //(RelativeOffset[i] / 360) * 55, // TODO figure out this math
              SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FeedForward);

    }
    SmartDashboard.putNumber("FL Setpoint", (OptimizedStates[0].angle.getRadians() * 360/(2*Math.PI)));
    jamesDebug();
    
  }

  /***************************************************************************************/
  /*                           OTHER FUNCTIONS                                           */
  /***************************************************************************************/
  /* Perform Kinematics to determine module commands */
  public SwerveModuleState[] performKinematicWith(double travelX, double travelY, double rotateRadians) {
    /* Create chassis speeds and module state objects */
    ChassisSpeeds speeds = new ChassisSpeeds(travelX, travelY, rotateRadians);
    SwerveModuleState[] moduleStates = Kinematics.toSwerveModuleStates(speeds);

    /* Optimize the angle */
    for (int i = 0; i < ModuleOrder.SizeOfModuleOrder.ordinal(); i++) {
      moduleStates[i] = angleMinimize(analogEncoders[i].getDegrees(), moduleStates[i]);
    }

    /* Account for max speed */
    Kinematics.desaturateWheelSpeeds(moduleStates, MaxDriveSpeed);
    return moduleStates;
  }

  /* Optimize the angle setpoint to ensure we never rotate more than 90 degrees */
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

  /* Initialize the analog input objects */
  public void AnalogInit() {
    // initialize the analog offset
    for (int i = 0; i < ModuleOrder.SizeOfModuleOrder.ordinal(); i++) {
      analogEncoders[i].offset = 0;// analogEncoders[i].maxVolt+analogEncoders[i].sensor.getPosition()-analogEncoders[i].zeroVolt;
      RelativeOffset[i] = analogEncoders[i].offset * 360 / analogEncoders[i].maxVolt;
    }
  }

  /* Create Smart Dashboard Elements for PID Tuning */
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

  /* Apply Smart Dashboard Elements for PID Tuning */
  public void PIDTuning() {
    double tuning = SmartDashboard.getNumber("PIDSetting: Steer(1) Drive(2) None(0)", 0);
    if (tuning != 1 || tuning != 2) // if tuning is None - do nothing
      return;
    double FF     = SmartDashboard.getNumber("Feed Forward", 0); // 0.000001
    double MaxOut = SmartDashboard.getNumber("Max Output", 1);
    double P      = SmartDashboard.getNumber("P Gain", 0); // 0.000170
    double I      = SmartDashboard.getNumber("I Gain", 0); // 0.000001
    double D      = SmartDashboard.getNumber("D Gain", 0); // 0.000020
    double IZ     = SmartDashboard.getNumber("I Zone", 0);
    double MinOut = SmartDashboard.getNumber("Min Output", -1);

    if (tuning == 1) {  // if tuning is steer
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
    } else if (tuning == 2) { // if tuning is drive
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
      VelocityBaseConfig.apply(VelocityLoopConfig);
    }

    /* Apply the updated tuning paramters */
    for (int i = 0; i < ModuleOrder.SizeOfModuleOrder.ordinal(); i++) {
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

  public void jamesDebug() {

    // Get the Angles
    for (int i = 0; i < ModuleOrder.SizeOfModuleOrder.ordinal(); i++)
    {
      AnalogEncoderDegrees[i] = analogEncoders[i].getDegrees();
    }

    SmartDashboard.putNumber("FL Angle", AnalogEncoderDegrees[ModuleOrder.FL.ordinal()]);
    SmartDashboard.putNumber("BL Angle", AnalogEncoderDegrees[ModuleOrder.BL.ordinal()]);
    SmartDashboard.putNumber("FR Angle", AnalogEncoderDegrees[ModuleOrder.FR.ordinal()]);
    SmartDashboard.putNumber("BR Angle", AnalogEncoderDegrees[ModuleOrder.BR.ordinal()]);
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
