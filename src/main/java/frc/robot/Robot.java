// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
//https://software-metadata.revrobotics.com/REVLib-2025.json
package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState.DRIVEORIENTATION;
import frc.robot.RobotState.MODE;
import frc.robot.commands.DetailedCommand;
import frc.robot.commands.DriveInfo;
import frc.robot.commands.ICommand;
import frc.robot.commands.Info;
import frc.robot.commands.LeftAlign;
import frc.robot.commands.ScoreInfo;

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

  public enum COMMAND {
    LeftAlign, RightAlign, Score, ClimbPrep, Climb, Drive
  }

  HashMap<COMMAND, ICommand> RegisteredCommands = new HashMap<COMMAND, ICommand>();
  DetailedCommand[] AutoCommands= new DetailedCommand[]{
    new DetailedCommand(COMMAND.Drive,new DriveInfo(5, 1, 1)),
    new DetailedCommand(COMMAND.LeftAlign, null),
    new DetailedCommand(COMMAND.Score, new ScoreInfo(4))
  };

  RobotState state;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    state = new RobotState();
    RegisteredCommands.put(COMMAND.LeftAlign, new LeftAlign());
    RegisteredCommands.put(COMMAND.RightAlign, null);
    RegisteredCommands.put(COMMAND.Score, null);
    RegisteredCommands.put(COMMAND.ClimbPrep, null);
    RegisteredCommands.put(COMMAND.Climb, null);
    RegisteredCommands.put(COMMAND.Drive, null);
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
    switch (m_autoSelected) {
      case kCustomAuto:
        state.mode = MODE.Auto1;
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    if(AutoCommands.length>state.commandIndex){
      COMMAND com = AutoCommands[state.commandIndex].command;
      Info in = AutoCommands[state.commandIndex].info;
      ICommand command =RegisteredCommands.get(com); 
      if((command.isActive(state)||command.shouldActivate(state, in))&&command.remainActive(state)){
          command.execute(state);
      }else{
        state.commandIndex++;
      }
    }else{
      /*
       * end autonomous mode
       */
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    state.Chassis.AnalogInit();
    state.mode= MODE.Teleop;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RegisteredCommands.forEach((COMMAND name, ICommand command) -> {
      if (command.isActive(state) || command.shouldActivate(state, null))
        if (command.remainActive(state))
          command.execute(state);
    });

    double JoystickTolerance = 0.09;
    if(!state.joystickControl)
      JoystickTolerance=0.7;
    double TranslateY = -state.JoystickL.getX();
    double TranslateX = -state.JoystickL.getY();
    double TranslateRotation = -state.JoystickR.getX();

    if (Math.abs(TranslateX) < JoystickTolerance)
      TranslateX = 0.0;
    if (Math.abs(TranslateY) < JoystickTolerance)
      TranslateY = 0.0;
    if (Math.abs(TranslateRotation) < JoystickTolerance)
      TranslateRotation = 0.0;

    if (TranslateX > 0 || TranslateY > 0 || TranslateRotation > 0){
      if(state.driveOrientation==DRIVEORIENTATION.Robot){
        state.Chassis.driveRobotOriented(TranslateX * state.Chassis.MaxDriveSpeed, TranslateY * state.Chassis.MaxDriveSpeed, TranslateRotation * state.Chassis.MaxTurnSpeed);
      }else{
        state.Chassis.driveFieldOriented(TranslateX * state.Chassis.MaxDriveSpeed, TranslateY * state.Chassis.MaxDriveSpeed, TranslateRotation * state.Chassis.MaxTurnSpeed);
      }
      state.joystickControl=true;
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
