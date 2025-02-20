package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveDrive;

public class RobotState {
    //controls
    public Joystick JoystickL = new Joystick(0);
    public Joystick JoystickR = new Joystick(1);
    // public PS4Controller controller = new PS4Controller(0);
    // public PS5Controller controller = new PS5Controller(0);
    // public XboxController controller = new XboxController(0);
    public SwerveDrive Chassis;

    //state constants
    public enum ALIGNMENT {
        None, Left, Right
    }
    public enum DRIVEORIENTATION{
        Robot, Field
    }
    public enum MODE{
        None,Teleop, Auto1
    }

    //state variables
    public ALIGNMENT alignment = ALIGNMENT.None;
    public DRIVEORIENTATION driveOrientation = DRIVEORIENTATION.Robot;
    public boolean joystickControl = true;
    public MODE mode= MODE.None;
    public int commandIndex=0;

    public RobotState(){
        Chassis= new SwerveDrive();
    }
}
