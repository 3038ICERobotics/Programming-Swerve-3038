// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.io.ObjectInputFilter.Config;
import java.lang.reflect.Array;
import java.security.Key;

import org.ejml.equation.Variable;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.config.BaseConfig;


//   [SWERVE IS IN METERS]   //
// BASE: 0.711m x 0.711m
// WHEELS FROM CENTER: 0.285m
//    [FRONT IS LIGHT]    //
// FLEFT: (0.285, 0.285)
// FRIGHT: (0.285, -0.285)
// BLEFT: (-0.285, 0.285)
// BRIGHT: (-0.285, -0.285)


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
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
  public ClosedLoopConfig VelocityLoopConfig  = new ClosedLoopConfig();
  public ClosedLoopConfig SteeringLoopConfig  = new ClosedLoopConfig();
  public SparkBaseConfig SteeringBaseConfig   = new SparkMaxConfig();
  public SparkBaseConfig VelocityBaseConfig   = new SparkMaxConfig();

  // Constants used to translate RPM to robot speed
  private final int RotationsPerMeter = 27;
  private final int SecondsPerMinute = 60;
  private final double MaxDriveSpeed = .1;
  private final double MaxTurnSpeed = .1;
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

double JoystickTolerance = 0.06;

  //Initialize Motors
  SparkMax FrontLeftDrive   =   new SparkMax(10, MotorType.kBrushless);
  SparkMax FrontLeftSteer   =   new SparkMax(15, MotorType.kBrushless);
  SparkMax FrontRightDrive  =   new SparkMax(3, MotorType.kBrushless);
  SparkMax FrontRightSteer  =   new SparkMax(1, MotorType.kBrushless);
  SparkMax BackLeftDrive    =   new SparkMax(22, MotorType.kBrushless);
  SparkMax BackLeftSteer    =   new SparkMax(8, MotorType.kBrushless);
  SparkMax BackRightDrive   =   new SparkMax(14, MotorType.kBrushless);
  SparkMax BackRightSteer   =   new SparkMax(2, MotorType.kBrushless);
  SparkMax[] motors = {
    FrontLeftDrive, 
    BackLeftDrive, 
    FrontRightDrive, 
    BackRightDrive, 
    FrontLeftSteer, 
    BackLeftSteer, 
    FrontRightSteer, 
    BackRightSteer};

  
  // Array
   private SparkClosedLoopController[] PIDControllers = //{null,null,null,null,null,null,null,null}; 
   {
      FrontLeftDrive.getClosedLoopController(),  BackLeftDrive.getClosedLoopController(),
      FrontRightDrive.getClosedLoopController(), BackRightDrive.getClosedLoopController(),
      FrontLeftSteer.getClosedLoopController(),  BackLeftSteer.getClosedLoopController(),
      FrontRightSteer.getClosedLoopController(), BackRightSteer.getClosedLoopController() 
      };
  public enum SwerveSparks {
    FLD, BLD, FRD, BRD, FLS, BLS, FRS, BRS
  };
  public enum ModuleOrder {
    FL, BL, FR, BR
  };

  //Initialize Encoders
  RelativeEncoder FrontLeftDriveEncoder = FrontLeftDrive.getEncoder();
  RelativeEncoder FrontLeftSteerEncoder = FrontLeftSteer.getEncoder(); // I don't think the absolute encoder works for this - James
  RelativeEncoder FrontRightDriveEncoder = FrontRightDrive.getEncoder();
  RelativeEncoder FrontRightSteerEncoder = FrontRightSteer.getEncoder();
  RelativeEncoder BackLeftDriveEncoder = BackLeftDrive.getEncoder();
  RelativeEncoder BackLeftSteerEncoder = BackLeftSteer.getEncoder();
  RelativeEncoder BackRightDriveEncoder = BackRightDrive.getEncoder();
  RelativeEncoder BackRightSteerEncoder = BackRightSteer.getEncoder();
  RelativeEncoder[] encoders = new RelativeEncoder[8];//{null,null,null,null,null,null,null,null};

  SparkAnalogSensor FrontLeftAnalog = FrontLeftSteer.getAnalog(); //Range 0-2.22 0 is 1.6
  SparkAnalogSensor FrontRightAnalog = FrontRightSteer.getAnalog();
  SparkAnalogSensor BackLeftAnalog = BackLeftSteer.getAnalog();
  SparkAnalogSensor BackRightAnalog = BackRightSteer.getAnalog();
  AnalogContainer[] analogs = new AnalogContainer[4];
//  SparkAnalogSensor[] analogs = new SparkAnalogSensor[4]; //{null,null,null,null};
  

  //Initialize Joystick
  Joystick JoystickL = new Joystick(0);
  Joystick JoystickR = new Joystick(1);
  Double TranslateX = 0.0;
  Double TranslateY = 0.0;
  Double TranslateRotation = 0.0;

  //Swerve Kinematics
  Translation2d FrontLeftDriveLocation = new Translation2d(0.285, 0.285);
  Translation2d FrontRightDriveLocation = new Translation2d(0.285, -0.285);
  Translation2d BackLeftDriveLocation = new Translation2d(-0.285, 0.285);
  Translation2d BackRightDriveLocation = new Translation2d(-0.285, -0.285);
  SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(FrontLeftDriveLocation, FrontRightDriveLocation, BackLeftDriveLocation, BackRightDriveLocation);

  SwerveModuleState frontLeft = new SwerveModuleState();
  SwerveModuleState frontRight = new SwerveModuleState();
  SwerveModuleState backLeft = new SwerveModuleState();
  SwerveModuleState backRight = new SwerveModuleState();

  SwerveModuleState frontLeftOptimized = new SwerveModuleState();
  SwerveModuleState frontRightOptimized = new SwerveModuleState();
  SwerveModuleState backLeftOptimized = new SwerveModuleState();
  SwerveModuleState backRightOptimized = new SwerveModuleState();
  SwerveModuleState[]OptimizedStates = new SwerveModuleState[4];

  double BLSTuningSetpoint = 0.0;

// // Convert to chassis speeds
 ChassisSpeeds chassisSpeeds = Kinematics.toChassisSpeeds(frontLeft, frontRight, backLeft, backRight);

// // Getting individual speeds
 double forward = chassisSpeeds.vxMetersPerSecond;
 double sideways = chassisSpeeds.vyMetersPerSecond;
 double angular = chassisSpeeds.omegaRadiansPerSecond;

//
  private double [] RelativeOffset = {
    0,
    0,
    0,
    0
  };

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    for(int i = 0; i<8; i++){
      //PIDControllers[i] = motors[i].getClosedLoopController();
      encoders[i] = motors[i].getEncoder();
      encoders[i].setPosition(0);
      if(i>3){
      }
    }
    analogs[0]=new AnalogContainer(motors[4].getAnalog(),2.23,1.60);
    analogs[1]=new AnalogContainer(motors[5].getAnalog(),2.23,2.18);
    analogs[2]=new AnalogContainer(motors[6].getAnalog(),2.27,1.78);
    analogs[3]=new AnalogContainer(motors[7].getAnalog(),2.20,0.12);



    Prop = 1;//1; //P = 0.000170
    Int = 0.5; //0.5; //I = 0.000001
    Der = 0.1;//0.1; //D = 0.000020
    IZone = 0.000005;//0.000005; //IZone = 0
    FeedForward = 0.00001;//0.00001; //FF = 0.000001
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
  
    VelocityLoopConfig.pidf(0.000170, 0.000001, 0.000020, 0.000001, ClosedLoopSlot.kSlot0);
    VelocityLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);

    SteeringLoopConfig.pidf(Prop, Int, Der, FeedForward, ClosedLoopSlot.kSlot0);
    SteeringLoopConfig.outputRange(MinOutput, MaxOutput, ClosedLoopSlot.kSlot0);

    VelocityBaseConfig.apply(VelocityLoopConfig);
    VelocityBaseConfig.inverted(false);
    SteeringBaseConfig.apply(SteeringLoopConfig);
    for(int i=0;i<8;i++){
      if(i<4){
        motors[i].configure(VelocityBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      } else{
        motors[i].configure(SteeringBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }

  //Velocity Loop Config
    
  

// // set PID coefficients
//     VelocityLoopController.
//     VelocityLoopController.setI(Int);
//     VelocityLoopController.setD(kD);
//     VelocityLoopController.setIZone(kIz);
//     VelocityLoopController.setFF(kFF);
//     VelocityLoopController.setOutputRange(kMinOutput, kMaxOutput);

    //Display PID coefficients on Dashboard
  

//  VoltageFL = FrontLeftAnalog.getVoltage();
//  PositionFL = FrontLeftAnalog.getPosition();
//  VoltageFR = FrontRightAnalog.getVoltage();
//  PositionFR = FrontRightAnalog.getPosition();
//  VoltageBL = BackLeftAnalog.getVoltage();
//  PositionBL = BackLeftAnalog.getPosition();
//  VoltageBR = BackRightAnalog.getVoltage();
//  PositionBR = BackRightAnalog.getPosition();

//  DegreeFL = -PositionFL * 180;//(-VoltageFL+2.228299140930176)%2.228299140930176*(360/2.228299140930176);
//  DegreeFR = -PositionFR * 180;//(-VoltageFR+2.287067413330078)%2.287067413330078*(360/2.287067413330078);
//  DegreeBL = -PositionBL * 180;//(-VoltageBL+2.233196496963)%2.233196496963*(360/2.233196496963);
//  DegreeBR = -PositionBR * 180;//(-VoltageBR+2.2136070728302)%2.2136070728302*(360/2.2136070728302);


RelativeOffset [ModuleOrder.FL.ordinal()] = 0;//DegreeFL;//(FrontLeftAnalog.getVoltage()+2.228299140930176-1.6014369726180)%2.228299140930176*(56/2.228299140930176);
RelativeOffset [ModuleOrder.BL.ordinal()] = 0;//DegreeBL; //(24.2/56) * (2*(Math.PI)) - BackLeftAnalog.getPosition(); //(BackLeftAnalog.getVoltage()+2.287067413330078-1.7973313331604)%2.287067413330078*(56/2.287067413330078) - BackLeftAnalog.getPosition(); // + (40.5/56);
RelativeOffset [ModuleOrder.FR.ordinal()] = 0;//DegreeFR;//(FrontRightAnalog.getVoltage()+2.233196496963-2.179325580596924)%2.233196496963*(56/2.233196496963);
RelativeOffset [ModuleOrder.BR.ordinal()] = 0;//DegreeBR;//(BackRightAnalog.getVoltage()+2.2136070728302-0.112639293074608)%2.2136070728302*(56/2.2136070728302);

}

public void AnalogInit(){
  //initialize the analog offset
  for(int i=0;i<4;i++){
    analogs[i].offset= 0;//analogs[i].maxVolt+analogs[i].sensor.getPosition()-analogs[i].zeroVolt;
    RelativeOffset[i]=analogs[i].offset*360/analogs[i].maxVolt;
  }
}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //[VARIABLES: 54700 max RPM, ~27 rotations per meter, 60 seconds per minute, ~3.5mps for max speed]

   ChassisSpeeds speeds = new ChassisSpeeds(TranslateX, TranslateY, TranslateRotation);
  // Convert to module states
   SwerveModuleState[] moduleStates = Kinematics.toSwerveModuleStates(speeds);
  // SwerveModuleState frontLeft = moduleStates[0];
  // SwerveModuleState frontRight = moduleStates[1];
  // SwerveModuleState backLeft = moduleStates[2];
  // SwerveModuleState backRight = moduleStates[3];

  //SwerveModuleState[]OptimizedStates = moduleStates;//{frontLeftOptimized, frontRightOptimized, backLeftOptimized, backRightOptimized};

  double[] currentAngles = new double[4];
  for(int i =0;i<4;i++){
    currentAngles[i] = analogs[i].getDegrees();//getAnalogAngleRotation(i);
    // OptimizedStates[i] = SwerveModuleState.optimize(moduleStates[i], new Rotation2d(currentAngles[i] * 2*Math.PI/360));
    OptimizedStates[i] = angleMinimize(currentAngles[i], moduleStates[i]);
  }

//Pass in all 4 optimized swerve module states as a list to Kinematics.desaturateWheelSpeeds 
//to normalize the speeds against the max
    Kinematics.desaturateWheelSpeeds(OptimizedStates, MaxDriveSpeed);
  }
  // public double getAnalogAngleRotation(int Index){

  //   return analogs[Index].getPosition() / 55 - RelativeOffset[Index] / 360;
  // }

  public SwerveModuleState angleMinimize(double CurrentAngle, SwerveModuleState TargetState){
    double newAngle = TargetState.angle.getDegrees() - CurrentAngle;

    if (newAngle > 90){
      newAngle -= 180;
      TargetState.speedMetersPerSecond *= -1;
    } else if (newAngle < -90){
      newAngle += 180;
      TargetState.speedMetersPerSecond *= -1;
    } 
    TargetState.angle = new Rotation2d((CurrentAngle + newAngle) * Math.PI/180);
        return TargetState;
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
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
    SmartDashboard.putNumber("frontLeft Zero", RelativeOffset[ModuleOrder.FL.ordinal()]);//-.25
    SmartDashboard.putNumber("frontRight Zero",RelativeOffset[ModuleOrder.FR.ordinal()]);//.9
    SmartDashboard.putNumber("backLeft Zero", RelativeOffset[ModuleOrder.BL.ordinal()]);//-.05
    SmartDashboard.putNumber("BackRight Zero", RelativeOffset[ModuleOrder.BR.ordinal()]);//.6
    AnalogInit();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  //Joystick Control
  TranslateY = -JoystickL.getX() * MaxDriveSpeed;
  TranslateX = -JoystickL.getY() * MaxDriveSpeed;
  TranslateRotation = -JoystickR.getX() * MaxTurnSpeed;

  //Once JoystickTolerance is found, substitute for numerical value: Easier to read for next year coders
  if (Math.abs(TranslateX) < JoystickTolerance) TranslateX = 0.0;
  if (Math.abs(TranslateY) < JoystickTolerance) TranslateY = 0.0;
  if (Math.abs(TranslateRotation) < JoystickTolerance) TranslateRotation = 0.0;

    // [MAXIMUM OBSERVED: 1.82 m/s] //
    //Setting drive speed
    //FrontLeftDrive.set(frontLeftOptimized.speedMetersPerSecond);
    //FrontRightDrive.set(frontRightOptimized.speedMetersPerSecond);
    //BackLeftDrive.set(backLeftOptimized.speedMetersPerSecond);
    //BackRightDrive.set(backRightOptimized.speedMetersPerSecond);
    // SmartDashboard.putNumber("MS", frontLeftOptimized.speedMetersPerSecond);
    
    SmartDashboard.putNumber("FLSE", FrontLeftSteerEncoder.getPosition());
    SmartDashboard.putNumber("BLSE", BackLeftSteerEncoder.getPosition());
    
    // //Velocity Loop numbers
    double P = SmartDashboard.getNumber("P Gain", 0); //0.000170
    double I = SmartDashboard.getNumber("I Gain", 0); //0.000001
    double D = SmartDashboard.getNumber("D Gain", 0); //0.000020
    double IZ = SmartDashboard.getNumber("I Zone", 0);
    double FF = SmartDashboard.getNumber("Feed Forward", 0); //0.000001
    double MaxOut = SmartDashboard.getNumber("Max Output", 1);
    double MinOut = SmartDashboard.getNumber("Min Output", -1);
    //Set SmartDashboard Numbers in PID
    // if (P != Prop) {
      SteeringLoopConfig.p(P, ClosedLoopSlot.kSlot0);
    // };
    // if (I != Int) {
      SteeringLoopConfig.i(I, ClosedLoopSlot.kSlot0);
    // };
    // if (D != Der) {
      SteeringLoopConfig.d(D, ClosedLoopSlot.kSlot0);
    // };
    // if (IZ != IZone) {
      SteeringLoopConfig.iZone(IZ, ClosedLoopSlot.kSlot0);
    // };
    // if (FF != FeedForward) {
      SteeringLoopConfig.velocityFF(FF, ClosedLoopSlot.kSlot0);
    // };
    // if ((MaxOut != MaxOutput) || (MinOut != MinOutput)) {
      SteeringLoopConfig.outputRange(MaxOut, MinOut, ClosedLoopSlot.kSlot0);
    // };


// // TODO - tune and come up with PID values for steering.
// SteeringBaseConfig
// .inverted(true);
SteeringBaseConfig.encoder
.positionConversionFactor(1)
.velocityConversionFactor(1);
SteeringBaseConfig.closedLoop
.pidf(P, I, D,FF,ClosedLoopSlot.kSlot0);
SteeringLoopConfig.outputRange(-1, 1);
SteeringBaseConfig.smartCurrentLimit(15); // Default limit is 80A - this limit is too high for a NEO 550

// VelocityBaseConfig.apply(VelocityLoopConfig);
SteeringBaseConfig.apply(SteeringLoopConfig);

for(int i=0;i<8;i++){
  if(i<4){
    motors[i].configure(VelocityBaseConfig,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }else{
    motors[i].configure(SteeringBaseConfig,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}

boolean toggle = SmartDashboard.getBoolean("toggle", false);
double[] setPoints = {
  OptimizedStates[0].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute,
  OptimizedStates[1].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute,
  OptimizedStates[2].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute,
  OptimizedStates[3].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute,
  OptimizedStates[0].angle.getRadians()*(toggle?0:1), //SmartDashboard.getNumber("frontLeft Zero", RelativeOffset[ModuleOrder.FL.ordinal()]),//-.35
  OptimizedStates[1].angle.getRadians()*(toggle?0:1), //- SmartDashboard.getNumber("backLeft Zero", RelativeOffset[ModuleOrder.BL.ordinal()]),//-.01
  OptimizedStates[2].angle.getRadians()*(toggle?0:1), //- SmartDashboard.getNumber("frontRight Zero", RelativeOffset[ModuleOrder.FR.ordinal()]),//2.27
  OptimizedStates[3].angle.getRadians()*(toggle?0:1), //- SmartDashboard.getNumber("BackRight Zero", RelativeOffset[ModuleOrder.BR.ordinal()])//.6
};

for (int i = 0; i<8; i++){ 
  if (i<4) {   //drive
    SmartDashboard.putString("TargetDrive Status"+ModuleOrder.values()[i].toString(), PIDControllers[i].setReference(setPoints[i], SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, FeedForward).toString());
    SmartDashboard.putNumber("TargetDrive"+ModuleOrder.values()[i].toString(), setPoints[i]);
  } else {
    SmartDashboard.putString("TargetSteer Status"+ModuleOrder.values()[i-4].toString(), PIDControllers[i].setReference(setPoints[i] * 55 / (2* Math.PI) + (RelativeOffset[i-4]/360)*55, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, FeedForward).toString());
    SmartDashboard.putNumber("TargetSteer"+ModuleOrder.values()[i-4].toString(), setPoints[i]*360/(2*Math.PI));
    SmartDashboard.putNumber("SteerCurAnalogOffset"+ModuleOrder.values()[i-4].toString(), RelativeOffset[i-4]);
    SmartDashboard.putNumber("SteerCurAnalogDirect"+ModuleOrder.values()[i-4].toString(), analogs[i-4].getDegrees());

    SmartDashboard.putNumber("SteerCurEnc"+ModuleOrder.values()[i-4].toString(), encoders[i].getPosition()/55);
//steer
  }
}
  //BLSTuningSetpoint = SmartDashboard.getNumber("BLSetpoint",0.0)/360;
  // for(int i =4; i<8;i++){
  //   SmartDashboard.putNumber("Target"+ModuleOrder.values()[i-4].toString(), (BLSTuningSetpoint + RelativeOffset[i-4]/360)*55);
  //   SmartDashboard.putString("Status"+ModuleOrder.values()[i-4].toString(), PIDControllers[i].setReference((BLSTuningSetpoint + RelativeOffset[i-4]/360)*55, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0).toString());
  //}
  //  SmartDashboard.putString("FL", PIDControllers[SwerveSparks.FLS.ordinal()].setReference((BLSTuningSetpoint + RelativeOffset[ModuleOrder.FL.ordinal()]/360)*55, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0).toString());
  //  SmartDashboard.putString("FR", PIDControllers[SwerveSparks.FRS.ordinal()].setReference((BLSTuningSetpoint + RelativeOffset[ModuleOrder.FR.ordinal()]/360)*55, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0).toString());
  //  SmartDashboard.putString("BR", PIDControllers[SwerveSparks.BRS.ordinal()].setReference((BLSTuningSetpoint + RelativeOffset[ModuleOrder.BR.ordinal()]/360)*55, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0).toString());
  //  SmartDashboard.putString("BL", PIDControllers[SwerveSparks.BLS.ordinal()].setReference((BLSTuningSetpoint + RelativeOffset[ModuleOrder.BL.ordinal()]/360)*55, SparkMax.ControlType.kPosition,ClosedLoopSlot.kSlot0).toString());
  
    SmartDashboard.putNumber("Speed", frontLeftOptimized.speedMetersPerSecond);
    SmartDashboard.putNumber("TestEncoder", FrontLeftSteerEncoder.getPosition());
    

    

    
  }
  
  //This is update all for steering directions and update all for drive velocities

  public void lowLevelDrive(){
  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

