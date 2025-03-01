package frc.robot;

import frc.robot.Elevator.ElevatorPositions;

public class RobotState {

    public boolean PickupAlgae = false;
    public boolean EjectAlgae = false;
    public boolean IntakeCoral = false;
    public boolean ScoreCoral = false;
    public int CurrentHeight = ElevatorPositions.Home.ordinal();
    public boolean ElevatorMoving = false;
}
