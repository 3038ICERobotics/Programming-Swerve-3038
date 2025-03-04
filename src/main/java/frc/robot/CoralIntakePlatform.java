package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class CoralIntakePlatform {

    SparkMax AngleLeft = new SparkMax(MotorIDs.AngleLeftID, MotorType.kBrushless);
    SparkMax AngleRight = new SparkMax(MotorIDs.AngleRightID, MotorType.kBrushless);
    SparkFlex Roller = new SparkFlex(MotorIDs.IntakeRollerID, MotorType.kBrushless);
    SparkMaxConfig LeftConfig = new SparkMaxConfig();
    SparkClosedLoopController PID;
    double FeedPosition = 0;
    double PrepPosition = 0;
    boolean InClimb = false;
    RelativeEncoder Encoder;
    SparkBaseConfig BaseConfig;
    DigitalInput BreakBeamClear = new DigitalInput(1);
    DigitalInput BreakBeamCoral = new DigitalInput(2);

    public CoralIntakePlatform(ClosedLoopConfig config) {
        LeftConfig.follow(AngleRight, true);
        PID = AngleRight.getClosedLoopController();
        BaseConfig = new SparkMaxConfig();
        BaseConfig.apply(config);
        AngleRight.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AngleLeft.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        Encoder = AngleRight.getEncoder();
    }

    public boolean GoToPosition(boolean GoToClimb){
        double TargetPosition = FeedPosition;
        if(GoToClimb){
            TargetPosition = PrepPosition;
        }
        PID.setReference(TargetPosition, ControlType.kPosition);

        return Encoder.getPosition() == TargetPosition;
    }

    public boolean Transfer(){
        boolean MotorFinish = BreakBeamClear.get() && !BreakBeamCoral.get();
        if(MotorFinish){
            Roller.set(0);
        }
        else{
            Roller.set(.5);
        }

        return MotorFinish;
    }
}
