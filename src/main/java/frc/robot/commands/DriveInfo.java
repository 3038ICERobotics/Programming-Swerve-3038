package frc.robot.commands;

public class DriveInfo extends Info {
    public double[] translations = new double[3];

    public DriveInfo(double X, double Y, double rot) {
        translations[0] = X;
        translations[1] = Y;
        translations[2] = rot;
    }
}
