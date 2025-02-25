package frc.robot;

import com.revrobotics.spark.SparkAnalogSensor;

public class AnalogContainer {
    public SparkAnalogSensor sensor;
    public double maxVolt;
    public double zeroVolt;
    public double offset;
    public double CalculatedAngle;
    public double CalculatedPosition;

    public AnalogContainer(SparkAnalogSensor s, double max, double zero) {
        sensor = s;
        maxVolt = max;
        zeroVolt = zero;
        CalculatedAngle = 0;
        CalculatedPosition = 0;
    }

    public double getDegrees() {
<<<<<<< Updated upstream
        return (((getRotation() * 360) + 180) % 360 ) - 180;
    }

    public double getRadians() {
        return getRotation() * 2 * Math.PI;
    }

    public double getZeroRotations() {
        return zeroVolt / maxVolt;
    }

    public double getZeroRadians() {
        return Math.PI * 2 * zeroVolt / maxVolt;
    }

    public double getRotation() {
        return ((maxVolt - zeroVolt + sensor.getPosition()) % maxVolt) / maxVolt;// (sensor.getPosition()-offset+maxVolt)%maxVolt;//-maxVolt/2;
    }

    public void setCalculatedAngle(double angle) {
        CalculatedAngle = angle;
    }

    public void setCalculatedPosition(double position) {
        CalculatedPosition = position;
    }

    public double getCalculatedAngle() {
        return CalculatedAngle;
    }

    public double getCalculatedPosition() {
        return CalculatedPosition;
    }
}
