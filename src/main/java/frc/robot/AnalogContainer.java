package frc.robot;

import com.revrobotics.spark.SparkAnalogSensor;

public class AnalogContainer {
    public SparkAnalogSensor sensor;
    public double maxVolt;
    public double zeroVolt;
    public double offset;

    public AnalogContainer(SparkAnalogSensor s, double max, double zero){
        sensor=s;
        maxVolt=max;
        zeroVolt=zero;
    }

    public double getDegrees(){
        return getRotation()*360;
    }

    public double getRotation(){
        return sensor.getPosition()/maxVolt;//(sensor.getPosition()-offset+maxVolt)%maxVolt;//-maxVolt/2;
    }
}
