package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaePickup {

    // ALGAE [ EDIT ]
    SparkMax AlgaeLeft = new SparkMax(MotorIDs.AlgaeLeftID, MotorType.kBrushless);
    SparkMax AlgaeRight = new SparkMax(MotorIDs.AlgaeRightID, MotorType.kBrushless);
    SparkFlex AlgaeFlex = new SparkFlex(MotorIDs.AlgaeFlexID, MotorType.kBrushless);
    SparkMaxConfig AlgaeLeftConfig = new SparkMaxConfig();
    SparkClosedLoopController AlgaePID;
    double HomePosition = 0;
    double PickUpPosition = 0;
    RelativeEncoder AlgaeEncoder;
    SparkBaseConfig BaseConfig;
    DigitalInput LimitSwitch = new DigitalInput(0); //Could be a limit switch

    public AlgaePickup(ClosedLoopConfig config) {
        AlgaeLeftConfig.follow(AlgaeRight, true);
        AlgaePID = AlgaeRight.getClosedLoopController();
        BaseConfig = new SparkMaxConfig();
        BaseConfig.apply(config);
        AlgaeRight.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlgaeLeft.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlgaeRight.getEncoder();
    }

    public boolean Pickup() {
        AlgaePID.setReference(PickUpPosition, ControlType.kPosition);
        if (LimitSwitch.get()) {
            AlgaeFlex.set(.5);
        } else {
            AlgaeFlex.set(0);
        }

        return AlgaeEncoder.getPosition() == PickUpPosition && !LimitSwitch.get();
    }

    public boolean Eject() {

        if (!LimitSwitch.get()) {
            AlgaeFlex.set(-.5);
        } else {
            AlgaePID.setReference(HomePosition, ControlType.kPosition);
            AlgaeFlex.set(0);
        }

        return AlgaeEncoder.getPosition() == HomePosition && LimitSwitch.get();
    }
}
