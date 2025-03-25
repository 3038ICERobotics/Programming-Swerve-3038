package frc.robot;

import frc.robot.Elevator.ElevatorPositions;

public class RobotState {

    public boolean KickAlgae = false;
    public boolean HomeAlgae = false;
    public boolean IntakeCoral = false;
    public boolean ScoreCoral = false;
    public int CurrentHeight = ElevatorPositions.Home.ordinal();
    public boolean ElevatorMoving = false;
    public boolean InClimbPrep = false;
    public boolean ClimbPrepInProgress = false;
    public boolean ClearCoral = false;
}
