package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaePickup {

    // ALGAE [ EDIT ]
    SparkMax AlgaeLeft = new SparkMax(MotorIDs.AlgaeLeftID, MotorType.kBrushless);
    SparkFlex AlgaeFlex = new SparkFlex(MotorIDs.AlgaeFlexID, MotorType.kBrushless);
    SparkMaxConfig AlgaeLeftConfig = new SparkMaxConfig();
    SparkClosedLoopController AlgaePID;
    double HomePosition = 0;
    double PickUpPosition = 5;
    RelativeEncoder AlgaeEncoder;
    SparkBaseConfig BaseConfig;
    DigitalInput LimitSwitch = new DigitalInput(3); // Could be a limit switch

    public AlgaePickup(ClosedLoopConfig config) {
        SmartDashboard.putNumber("AlgaePickup Setpoint", 0);
        BaseConfig = new SparkMaxConfig();
        BaseConfig.apply(config);
        BaseConfig.idleMode(IdleMode.kCoast);
        AlgaeLeft.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlgaeEncoder = AlgaeLeft.getEncoder();
        AlgaeEncoder.setPosition(0);
        AlgaePID = AlgaeLeft.getClosedLoopController();
    }

    public void DisplayPosition() {
        SmartDashboard.putNumber("AlgaeEncoder", AlgaeEncoder.getPosition());
        SmartDashboard.putBoolean("Algae Breakbeam", LimitSwitch.get());

    }

    public void Test() {
        double Setpoint = SmartDashboard.getNumber("AlgaePickup Setpoint", 0);
        AlgaePID.setReference(Setpoint, ControlType.kPosition);

    }

    public void UpdatePID(ClosedLoopConfig config) {
        BaseConfig.apply(config);
        AlgaeLeft.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // return true to continue, false to stop
    public boolean HomeAlgae() {
        boolean result = LimitSwitch.get();
        if (result) {
            AlgaeLeft.set(-.08);
        } else {
            AlgaeLeft.set(0);
        }
        AlgaeFlex.set(0);
        return result;
    }

    // return true to continue, false to stop
    public boolean KickAlgae() {
        boolean result = LimitSwitch.get();
        if (!result) {
            AlgaeLeft.set(.08);
        } else {
            AlgaeLeft.set(0);
        }
        AlgaeFlex.set(.6);
        return !result;
    }
}
