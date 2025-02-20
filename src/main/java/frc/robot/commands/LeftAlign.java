package frc.robot.commands;

import frc.robot.RobotState;
import frc.robot.RobotState.ALIGNMENT;

public class LeftAlign implements ICommand{
    double[] driveTargets= new double[4];
    double[] steerTargets= new double[4];
    boolean aprilTagRead= false;
    

    @Override
    public void execute(RobotState state) {
        /* 
         * call setReference for each of the motors with recorded Targets.
         */
    }

    @Override
    public boolean remainActive(RobotState state) {
        if(state.alignment==ALIGNMENT.Left){
            if(!aprilTagRead){
                /* 
                * Read April tag
                * if April tag can be read:
                *  Get distance, heading, and rotation
                *  convert into setpoint offsets
                *  add the offset to the current encoder postions and 
                *  set as the targets.
                *  set aprilTagRead = true;
                * else
                *  return false;
                */
            }
            /* 
             * if encoder positions are not equal to targets (with tolerance)
             *  return true
             */
        }
        return false;
    }

    @Override
    public boolean shouldActivate(RobotState state, Info info) {
        if(state.JoystickL.getRawButtonPressed(1)){
            state.alignment= ALIGNMENT.Left;
            state.joystickControl= false;
            aprilTagRead= false;
            return true;
        }
        return false;
    }

    @Override
    public boolean isActive(RobotState state) {
        return state.alignment==ALIGNMENT.Left;
    }

}
