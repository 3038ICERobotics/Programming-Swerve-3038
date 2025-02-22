// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveClass.ModuleOrder;

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

  // SparkAnalogSensor[] analogs = new SparkAnalogSensor[4];
  // //{null,null,null,null};

  // Initialize Joystick
  Joystick JoystickL = new Joystick(0);
  Joystick JoystickR = new Joystick(1);
  double JoystickTolerance = 0.06;

  SwerveClass chassis = new SwerveClass();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    chassis.init();

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
          chassis.encoders[i + 4].getPosition() * 360 / 55);
      SmartDashboard.putNumber("Absolute Rotations" + ModuleOrder.values()[i].toString(),
          chassis.analogs[i].getDegrees());
      SmartDashboard.putNumber("Relative Offset" + ModuleOrder.values()[i].toString(),
          chassis.analogs[i].offset * 360 / 55);
    }
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
    // SmartDashboard.putNumber("frontLeft Zero",
    // RelativeOffset[ModuleOrder.FL.ordinal()]);// -.25
    // SmartDashboard.putNumber("frontRight Zero",
    // RelativeOffset[ModuleOrder.FR.ordinal()]);// .9
    // SmartDashboard.putNumber("backLeft Zero",
    // RelativeOffset[ModuleOrder.BL.ordinal()]);// -.05
    // SmartDashboard.putNumber("BackRight Zero",
    // RelativeOffset[ModuleOrder.BR.ordinal()]);// .6
    chassis.AnalogInit();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Joystick Control
    double TranslateY = -JoystickL.getX();
    double TranslateX = -JoystickL.getY();
    double TranslateRotation = -JoystickR.getX();

    if (Math.abs(TranslateX) < JoystickTolerance)
      TranslateX = 0.0;
    if (Math.abs(TranslateY) < JoystickTolerance)
    if (Math.abs(TranslateRotation) < JoystickTolerance)
      TranslateRotation = 0.0;

    chassis.drive(TranslateY * chassis.MaxDriveSpeed, TranslateX * chassis.MaxDriveSpeed,
        TranslateRotation * chassis.MaxTurnSpeed);
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
