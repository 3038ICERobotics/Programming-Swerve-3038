// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
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
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Elevator.ElevatorPositions;

//   [SWERVE IS IN METERS]   
// BASE: 0.711m x 0.711m
// WHEELS FROM CENTER: 0.285m
//    [FRONT IS LIGHT]    
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
  SparkBaseConfig VelocityBaseConfig[] = new SparkBaseConfig[4];
  TalonFXConfiguration DriveConfig = new TalonFXConfiguration();
  ClosedLoopConfig Neo550 = new ClosedLoopConfig();

  // Constants used to translate RPM to robot speed
  private final int RotationsPerMeter = 27;
  private final int SecondsPerMinute = 60;
  // MaxDriveSpeed and MaxTurnSpeed is in meters per second
  private final double MaxDriveSpeed = 2;
  private final double MaxTurnSpeed = 3;
  double VoltageFL = 0;
  double PositionFL = 0;
  double VoltageFR = 0;
  double PositionFR = 0;
  double VoltageBL = 0;
  double PositionBL = 0;
  double VoltageBR = 0;
  double PositionBR = 0;

  double DegreeFL = 0;
  double DegreeFR = 0;
  double DegreeBL = 0;
  double DegreeBR = 0;

  double JoystickTolerance = 0.09;
  double GearRatio = 54.8;

  // Initialize Motors [EDIT TALONS]
  TalonFX FrontLeftDrive = new TalonFX(MotorIDs.FrontLeftDriveID);
  SparkMax FrontLeftSteer = new SparkMax(MotorIDs.FrontLeftSteerID, MotorType.kBrushless);
  TalonFX FrontRightDrive = new TalonFX(MotorIDs.FrontRightDriveID);
  SparkMax FrontRightSteer = new SparkMax(MotorIDs.FrontRightSteerID, MotorType.kBrushless);
  TalonFX BackLeftDrive = new TalonFX(MotorIDs.BackLeftDriveID);
  SparkMax BackLeftSteer = new SparkMax(MotorIDs.BackLeftSteerID, MotorType.kBrushless);
  TalonFX BackRightDrive = new TalonFX(MotorIDs.BackRightDrive);
  SparkMax BackRightSteer = new SparkMax(MotorIDs.BackRightSteerID, MotorType.kBrushless);
  //HOOK [ EDIT ]
  SparkMax Hook = new SparkMax(MotorIDs.HookID, MotorType.kBrushless);
  //INTAKE [ EDIT ]
  SparkFlex IntakeRoller = new SparkFlex(MotorIDs.IntakeRollerID, MotorType.kBrushless);
  SparkMax AngleLeft = new SparkMax(MotorIDs.AngleLeftID, MotorType.kBrushless);
  SparkMax AngleRight = new SparkMax(MotorIDs.AngleRightID, MotorType.kBrushless);

  // Motor Array
  TalonFX[] DriveMotors = {
      FrontLeftDrive,
      BackLeftDrive,
      FrontRightDrive,
      BackRightDrive
  };
  SparkMax[] SteerMotors = {
      FrontLeftSteer,
      BackLeftSteer,
      FrontRightSteer,
      BackRightSteer
  };

  VelocityVoltage[] TalonVoltage = {
    new VelocityVoltage(0).withSlot(0),
    new VelocityVoltage(0).withSlot(0),
    new VelocityVoltage(0).withSlot(0),
    new VelocityVoltage(0).withSlot(0) };

  // PID Controllers Array
  private SparkClosedLoopController[] PIDDriveControllers = new SparkClosedLoopController[4];
  private SparkClosedLoopController[] PIDSteerControllers = new SparkClosedLoopController[4];

  public enum SwerveSparks {
    FLD, BLD, FRD, BRD, FLS, BLS, FRS, BRS
  };

  public enum ModuleOrder {
    FL, BL, FR, BR
  };

  // Initialize Encoders
  RelativeEncoder[] encoders = new RelativeEncoder[8];

  // Initialize Analogs
  AnalogContainer[] analogs = new AnalogContainer[4];

  // Initialize Joystick
  Joystick JoystickL = new Joystick(0);
  Joystick JoystickR = new Joystick(1);
  Double TranslateX = 0.0;
  Double TranslateY = 0.0;
  Double TranslateRotation = 0.0;

  // Swerve Kinematics
  Translation2d FrontLeftDriveLocation = new Translation2d(-0.285, 0.285);
  Translation2d FrontRightDriveLocation = new Translation2d(-0.285, -0.285);
  Translation2d BackLeftDriveLocation = new Translation2d(0.285, 0.285);
  Translation2d BackRightDriveLocation = new Translation2d(0.285, -0.285);
  SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(FrontLeftDriveLocation, BackLeftDriveLocation,
      FrontRightDriveLocation, BackRightDriveLocation);

  SwerveModuleState frontLeft = new SwerveModuleState();
  SwerveModuleState frontRight = new SwerveModuleState();
  SwerveModuleState backLeft = new SwerveModuleState();
  SwerveModuleState backRight = new SwerveModuleState();

  SwerveModuleState frontLeftOptimized = new SwerveModuleState();
  SwerveModuleState frontRightOptimized = new SwerveModuleState();
  SwerveModuleState backLeftOptimized = new SwerveModuleState();
  SwerveModuleState backRightOptimized = new SwerveModuleState();
  SwerveModuleState[] OptimizedStates = new SwerveModuleState[4];

  Elevator ElevatorObject = new Elevator();
  AlgaePickup AlgaeGrabber;
  CoralIntakePlatform Intake;
  RobotState State = new RobotState();

  double BLSTuningSetpoint = 0.0;

  //Camera
  NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
    IntegerArraySubscriber pubTags = tagsTable.getIntegerArrayTopic("tags").;

  // Convert to chassis speeds
  // ChassisSpeeds chassisSpeeds = Kinematics.toChassisSpeeds(frontLeft, backLeft,
  // frontRight, backRight);

  // // Getting individual speeds
  // double forward = chassisSpeeds.vxMetersPerSecond;
  // double sideways = chassisSpeeds.vyMetersPerSecond;
  // double angular = chassisSpeeds.omegaRadiansPerSecond;

  // Offset Array
  private double[] RelativeOffset = {
      0,
      0,
      0,
      0
  };

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Neo550.pidf(1, .5, .1, .00001);
    AlgaeGrabber = new AlgaePickup(Neo550);
    Intake = new CoralIntakePlatform(Neo550);

    for (int i = 0; i < 4; i++) {
      // PIDDriveControllers[i] = DriveMotors[i];
      // encoders[i] = DriveMotors[i].getEncoder();
      PIDSteerControllers[i] = SteerMotors[i].getClosedLoopController();
      encoders[i + 4] = SteerMotors[i].getEncoder();
      if (i > 3) {
      }
    }
    analogs[0] = new AnalogContainer(SteerMotors[0].getAnalog(), 2.23, 1.60);
    analogs[1] = new AnalogContainer(SteerMotors[1].getAnalog(), 2.23, 2.18);
    analogs[2] = new AnalogContainer(SteerMotors[2].getAnalog(), 2.27, 1.78);
    analogs[3] = new AnalogContainer(SteerMotors[3].getAnalog(), 2.20, 0.12);

    // PID Values
    Prop = 1;// 1; //P = 0.000170
    Int = 0.5; // 0.5; //I = 0.000001
    Der = 0.1;// 0.1; //D = 0.000020
    IZone = 0.000005;// 0.000005; //IZone = 0
    FeedForward = 0.00001;// 0.00001; //FF = 0.000001
    MaxOutput = 1;
    MinOutput = -1;
    MaxRPM = 5700;

    SmartDashboard.putNumber("P Gain", Prop);
    SmartDashboard.putNumber("I Gain", Int);
    SmartDashboard.putNumber("D Gain", Der);
    SmartDashboard.putNumber("I Zone", IZone);
    SmartDashboard.putNumber("Feed Forward", FeedForward);
    SmartDashboard.putNumber("Max Output", MaxOutput);
    SmartDashboard.putNumber("Min Output", MinOutput);
    SmartDashboard.putNumber("BLSetpoint", 0);

    double TalonkP = DriveConfig.Slot0.kP;
    double TalonkI = DriveConfig.Slot0.kI;
    double TalonkD = DriveConfig.Slot0.kD;
    double TalonkS = DriveConfig.Slot0.kS;
    double TalonkV = DriveConfig.Slot0.kV;

    DriveConfig.Slot0.kP = 0.000170;
    DriveConfig.Slot0.kI = 0.000001;
    DriveConfig.Slot0.kD = 0.000020;
    DriveConfig.Slot0.kS = 0.000001;
    DriveConfig.Slot0.kV = 0.000001;

    SmartDashboard.putNumber("Talon P Gain", TalonkP);
    SmartDashboard.putNumber("Talon I Gain", TalonkI);
    SmartDashboard.putNumber("Talon D Gain", TalonkD);
    SmartDashboard.putNumber("Talon SFF", TalonkS);
    SmartDashboard.putNumber("Talon VFF", TalonkV);

    // Loop Configs
    VelocityLoopConfig.pidf(0.000170, 0.000001, 0.000020, 0.000001, ClosedLoopSlot.kSlot0);
    VelocityLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);

    SteeringLoopConfig.pidf(Prop, Int, Der, FeedForward, ClosedLoopSlot.kSlot0);
    SteeringLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);

    // Applying Configs
    SteeringBaseConfig.apply(SteeringLoopConfig);
    SteeringBaseConfig.signals.analogPositionPeriodMs(10);
    for (int i = 0; i < 4; i++) {
      VelocityBaseConfig[i] = new SparkMaxConfig();
      VelocityBaseConfig[i].apply(VelocityLoopConfig);
      VelocityBaseConfig[i].inverted(false);

      if (i == 1) {
        VelocityBaseConfig[i].inverted(true); }

      //DriveMotors[i].configure(VelocityBaseConfig[i], ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      SteerMotors[i].configure(SteeringBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    RelativeOffset[ModuleOrder.FL.ordinal()] = 0;
    RelativeOffset[ModuleOrder.BL.ordinal()] = 0;
    RelativeOffset[ModuleOrder.FR.ordinal()] = 0;
    RelativeOffset[ModuleOrder.BR.ordinal()] = 0;
  }

  public void AnalogInit() {
    // initialize the analog offset
    for (int i = 0; i < 4; i++) {
      analogs[i].offset = 0;
      RelativeOffset[i] = analogs[i].offset * 360 / analogs[i].maxVolt;
      analogs[i].offset = analogs[i].getRotation() * -GearRatio;
      encoders[i + 4].setPosition(analogs[i].getRotation() * -GearRatio);
      analogs[i].setCalculatedPosition(analogs[i].getRotation() * -GearRatio);
      analogs[i].setCalculatedAngle(analogs[i].getDegrees());
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
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Relative Rotations" + ModuleOrder.values()[i].toString(),
        encoders[i + 4].getPosition());
      SmartDashboard.putNumber("Absolute Rotations" + ModuleOrder.values()[i].toString(), analogs[i].getDegrees());
      SmartDashboard.putNumber("Relative Offset" + ModuleOrder.values()[i].toString(),
          analogs[i].offset * 360 / GearRatio);
      SmartDashboard.putNumber("Position" + ModuleOrder.values()[i].toString(), analogs[i].sensor.getPosition());
    }
    // [VARIABLES: 54700 max RPM, ~27 rotations per meter, 60 seconds per minute,
    // ~3.5mps for max speed]
  }

  private void PerformKinematics() {

    // Swerve Speed variables
    ChassisSpeeds speeds = new ChassisSpeeds(TranslateX, TranslateY, TranslateRotation);

    // Convert to module states
    SwerveModuleState[] moduleStates = Kinematics.toSwerveModuleStates(speeds);

    double[] currentAngles = new double[4];
    SmartDashboard.putNumber("BackLeftTestyThingy", moduleStates[ModuleOrder.BL.ordinal()].angle.getDegrees());

    for (int i = 0; i < 4; i++) {
      OptimizedStates[i] = angleMinimize(currentAngles[i], moduleStates[i], i);
    }
 
    // Pass in all 4 optimized swerve module states as a list to
    // Kinematics.desaturateWheelSpeeds
    // to normalize the speeds against the max
    Kinematics.desaturateWheelSpeeds(OptimizedStates, MaxDriveSpeed);
  }

  public double[] BoundaryCorrection(SwerveModuleState[] CommandStates, double[] CurrentAngle) {
    double[] DeltaAngles = new double[4];

    for (int i = 0; i < 4; i++) {
      DeltaAngles[i] = CommandStates[i].angle.getDegrees();// - CurrentAngle[i];
      SmartDashboard.putNumber("deltaAngle", DeltaAngles[i]);

      // if(DeltaAngles[i] > 180){
      // DeltaAngles[i] = 360 - DeltaAngles[i];
      // } else if(DeltaAngles[i] < -180){
      // DeltaAngles[i] = -DeltaAngles[i] - 360;
      // }
    }

    return DeltaAngles;
  }

  public SwerveModuleState angleMinimize(double CurrentAngle, SwerveModuleState TargetState, int ModuleIndex) {
    double deltaAngle = TargetState.angle.getDegrees() - analogs[ModuleIndex].CalculatedAngle;

    /*
     * Issue is that the current angle is not consistent with the target angle.
     * compare smart dashboard plots of the Target angle, current angle, and new
     * angle
     * We need to make sure they are consistent before calculating a delta.
     */

    if (deltaAngle > 90) {
      deltaAngle -= 180;
      TargetState.speedMetersPerSecond *= -1;
    } else if (deltaAngle < -90) {
      deltaAngle += 180;
      TargetState.speedMetersPerSecond *= -1;
    }

    if (deltaAngle > 90) {
      deltaAngle -= 180;
      TargetState.speedMetersPerSecond *= -1;
    } else if (deltaAngle < -90) {
      deltaAngle += 180;
      TargetState.speedMetersPerSecond *= -1;
    }
    
    TargetState.angle = new Rotation2d(
        ((deltaAngle) % 360) * Math.PI / 180);
    
    return TargetState;
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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //pubTags.getTopic().

    CheckButtonPresses();
    PerformActions();

    // Joystick Control
    TranslateY = JoystickL.getX() * Math.abs(-JoystickL.getX()) * MaxDriveSpeed;
    TranslateX = -JoystickL.getY() * Math.abs(-JoystickL.getY()) * MaxDriveSpeed;
    TranslateRotation = -JoystickR.getX() * MaxTurnSpeed;

    if (Math.abs(TranslateX) < JoystickTolerance)
      TranslateX = 0.0;
    if (Math.abs(TranslateY) < JoystickTolerance)
      TranslateY = 0.0;
    if (Math.abs(TranslateRotation) < JoystickTolerance)
      TranslateRotation = 0.0;

    PerformKinematics();

    double[] AngleList = new double[4];
    for (int i = 0; i < 4; i++) {
      AngleList[i] = analogs[i].getCalculatedAngle();
    }
    double[] DeltaAngles = BoundaryCorrection(OptimizedStates, AngleList);
    // [MAXIMUM OBSERVED: 1.82 m/s] //

    PIDTuning();

    applyDrive(DeltaAngles);
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("Angle Setpoint" + ModuleOrder.values()[i].toString(),
          OptimizedStates[i].angle.getDegrees());
      SmartDashboard.putNumber("Angle Calculated" + ModuleOrder.values()[i].toString(),
          analogs[i].getCalculatedAngle());
      SmartDashboard.putNumber("Delta Angles" + ModuleOrder.values()[i].toString(), DeltaAngles[i]);

    }
  }

  private void CheckButtonPresses() {
    //Buttons for Algae
    if (JoystickL.getRawButtonPressed(8)){
      State.PickupAlgae = true;
      State.EjectAlgae = false;
    }
    if (JoystickL.getRawButtonPressed(9)){
      State.PickupAlgae = false;
      State.EjectAlgae = true;
    }
    if (JoystickR.getRawButtonPressed(11)) {
      State.IntakeCoral = true; 
      State.ScoreCoral = false;
    }
    if (JoystickR.getRawButtonPressed(10)) {
      State.IntakeCoral = false; 
      State.ScoreCoral = true;
    }
    if (JoystickR.getRawButtonPressed(3)) {
      State.CurrentHeight = ElevatorPositions.Tray.ordinal();
      State.ElevatorMoving = true;
    }
    if (JoystickR.getRawButtonPressed(4)) {
      State.CurrentHeight = ElevatorPositions.First.ordinal();
      State.ElevatorMoving = true;
    }
    if (JoystickR.getRawButtonPressed(2)) {
      State.CurrentHeight = ElevatorPositions.Second.ordinal();
      State.ElevatorMoving = true;
    }
    if (JoystickR.getRawButtonPressed(5)) {
      State.CurrentHeight = ElevatorPositions.Third.ordinal();
      State.ElevatorMoving = true;
    }
    if(JoystickR.getRawButtonPressed(10)){
      State.InClimbPrep = !State.InClimbPrep;
      State.ClimbPrepInProgress = true;
    }
  }

  public void PerformActions(){
    if (State.PickupAlgae) {
      State.PickupAlgae = !AlgaeGrabber.Pickup();
    }
    if (State.EjectAlgae) {
      State.EjectAlgae = !AlgaeGrabber.Eject();
    }
    if (State.IntakeCoral) {
      State.IntakeCoral = !ElevatorObject.IntakeCoral() || !Intake.Transfer();
    }
    if (State.ScoreCoral) {
      State.ScoreCoral = !ElevatorObject.ScoreCoral();
    }
    if (State.ElevatorMoving) {
      State.ElevatorMoving = !ElevatorObject.GoToHeight(State.CurrentHeight);
    }
    if(State.ClimbPrepInProgress){
      State.ClimbPrepInProgress = !Intake.GoToPosition(State.InClimbPrep);
    }
  }

  private void PIDTuning() {

    // //Velocity Loop numbers
    double P = SmartDashboard.getNumber("P Gain", 0); // 0.000170
    double I = SmartDashboard.getNumber("I Gain", 0); // 0.000001
    double D = SmartDashboard.getNumber("D Gain", 0); // 0.000020
    double IZ = SmartDashboard.getNumber("I Zone", 0);
    double FF = SmartDashboard.getNumber("Feed Forward", 0); // 0.000001
    double MaxOut = SmartDashboard.getNumber("Max Output", 1);
    double MinOut = SmartDashboard.getNumber("Min Output", -1);

    // Steering Loop PID Values
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

    double TalonkP = SmartDashboard.getNumber("TalonkP", 0);
    double TalonkI = SmartDashboard.getNumber("TalonkI", 0);
    double TalonkD = SmartDashboard.getNumber("TalonkD", 0);
    double TalonkS = SmartDashboard.getNumber("TalonkS", 0);
    double TalonkV = SmartDashboard.getNumber("TalonkV", 0);

    DriveConfig.Slot0.kP = TalonkP;
    DriveConfig.Slot0.kI = TalonkI;
    DriveConfig.Slot0.kD = TalonkD;
    DriveConfig.Slot0.kS = TalonkS;
    DriveConfig.Slot0.kV = TalonkV;

    for (int i = 0; i < 4; i++) {
        //DriveMotors[i].configure(VelocityBaseConfig[i], ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SteerMotors[i].configure(SteeringBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  private void applyDrive(double[] DeltaAngles) {
    double[] DriveSetPoints = {
        OptimizedStates[0].speedMetersPerSecond,
        OptimizedStates[1].speedMetersPerSecond,
        OptimizedStates[2].speedMetersPerSecond,
        OptimizedStates[3].speedMetersPerSecond,
    }; double[] SteerSetPoints = {
        OptimizedStates[0].angle.getRotations(),
        OptimizedStates[1].angle.getRotations(),
        OptimizedStates[2].angle.getRotations(),
        OptimizedStates[3].angle.getRotations()
    };

    for (int i = 0; i < 4; i++) {
        SteerSetPoints[i] = (DeltaAngles[i] * -GearRatio / 360) + analogs[i].getCalculatedPosition();
      if (Math.abs(DriveSetPoints[i]) < .1) {
        DriveSetPoints[i] = 0;
      }
    }

    for (int i = 0; i < 4; i++) {
        SmartDashboard.putString("TargetDrive Status" + ModuleOrder.values()[i].toString(),
        DriveMotors[i].setControl(TalonVoltage[i].withVelocity(DriveSetPoints[i] * RotationsPerMeter * SecondsPerMinute)).toString());
        SmartDashboard.putNumber("TargetDrive" + ModuleOrder.values()[i].toString(), DriveSetPoints[i]);
        SmartDashboard.putString("TargetSteer Status" + ModuleOrder.values()[i].toString(), PIDSteerControllers[i]
            .setReference(SteerSetPoints[i], SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FeedForward).toString());
        analogs[i].CalculatedPosition = SteerSetPoints[i];
        analogs[i].CalculatedAngle += DeltaAngles[i];
        if (analogs[i].CalculatedAngle > 180) {
          analogs[i].CalculatedAngle -= 360;
        } else if (analogs[i].CalculatedAngle < -180) {
          analogs[i].CalculatedAngle += 360;
        }
    }
  }

  // This is update all for steering directions and update all for drive
  // velocities

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
