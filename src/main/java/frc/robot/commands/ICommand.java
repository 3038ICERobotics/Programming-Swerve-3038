package frc.robot.commands;

import frc.robot.RobotState;

public interface ICommand {
    /*
     * Returns if this command is currently active
     */
    public boolean isActive(RobotState state);

    /* Checks any conditions required for the specific command and 
     * returns true if this command should begin.
     * This can also make updates to the RobotState as necessary.
     */
    public boolean shouldActivate(RobotState state, Info info);

    /* Checks any conditions necessary to determine if the endpoint 
     * of the command has been reached and returns true if the endpoint
     * has not been reached yet.
     * This can also make updates to the RobotState as necesary.
     */
    public boolean remainActive(RobotState state);

    /* Performs the actual action relevant to this command. */
    public void execute(RobotState state);
}
