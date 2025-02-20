package frc.robot.commands;

import frc.robot.Robot.COMMAND;

public class DetailedCommand {
    public COMMAND command;
    public Info info;

    public DetailedCommand(COMMAND c, Info i) {
        command = c;
        info = i;
    }
}
