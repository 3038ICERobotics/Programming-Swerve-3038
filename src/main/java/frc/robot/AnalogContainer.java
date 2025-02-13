package frc.robot;

import com.revrobotics.spark.SparkAnalogSensor;

public class AnalogContainer {
    public SparkAnalogSensor sensor;
    public double maxVolt;
    public double zeroVolt;
    public double offset;
    public double zeroDegOffset;

    public AnalogContainer(SparkAnalogSensor s, double max, double zero){
        sensor=s;
        maxVolt=max;
        zeroVolt=zero;
        zeroDegOffset=zero/max * 360.0;
    }

    public double getDegrees(){
        return (((maxVolt-zeroVolt + sensor.getPosition()) % maxVolt) / maxVolt * 360.0);
    }

    public double getRotation(){
        return ((sensor.getPosition()/maxVolt)*360.0 - zeroDegOffset) % 360.0;//sensor.getPosition()/maxVolt;//(sensor.getPosition()-offset+maxVolt)%maxVolt;//-maxVolt/2;
    }
}
