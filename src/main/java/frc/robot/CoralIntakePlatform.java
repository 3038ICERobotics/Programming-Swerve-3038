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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralIntakePlatform {

    SparkMax AngleLeft = new SparkMax(MotorIDs.AngleLeftID, MotorType.kBrushless);
    SparkMax AngleRight = new SparkMax(MotorIDs.AngleRightID, MotorType.kBrushless);
    // SparkFlex Roller = new SparkFlex(MotorIDs.IntakeRollerID,
    // MotorType.kBrushless);
    SparkMaxConfig LeftConfig = new SparkMaxConfig();
    SparkClosedLoopController PID;
    double FeedPosition = 0;
    double PrepPosition = 0;
    boolean InClimb = false;
    RelativeEncoder Encoder;
    RelativeEncoder Encoder2;
    SparkBaseConfig BaseConfig;
    // DigitalInput BreakBeamClear = new DigitalInput(6);
    // DigitalInput BreakBeamCoral = new DigitalInput(2);

    public CoralIntakePlatform(ClosedLoopConfig config) {
        LeftConfig.follow(AngleRight, true);
        PID = AngleRight.getClosedLoopController();
        BaseConfig = new SparkMaxConfig();
        BaseConfig.apply(config);
        AngleRight.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AngleLeft.configure(LeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        Encoder = AngleRight.getEncoder();
        Encoder2 = AngleLeft.getEncoder();
        Encoder.setPosition(0);
        Encoder2.setPosition(0);
    }

    public void UpdatePID(ClosedLoopConfig config) {
        BaseConfig.apply(config);
        AngleRight.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void DisplayPosition() {
        SmartDashboard.putNumber("AngleLeftEncoder", Encoder.getPosition());
        SmartDashboard.putNumber("AngleRightEncoder", Encoder2.getPosition());

    }

    public void Test() {
        double Setpoint = SmartDashboard.getNumber("CoralIntake Setpoint", 0);
        PID.setReference(Setpoint, ControlType.kPosition);
        SmartDashboard.putNumber("CoralIntake Setpoint", Setpoint);
    }

    public boolean GoToPosition(boolean GoToClimb) {
        double TargetPosition = FeedPosition;
        if (GoToClimb) {
            TargetPosition = PrepPosition;
        }
        PID.setReference(TargetPosition, ControlType.kPosition);

        return Encoder.getPosition() == TargetPosition;
    }

    // public boolean Transfer(){
    // boolean MotorFinish = false; //BreakBeamClear.get(); //Fix this logic!!
    // if(MotorFinish){
    // Roller.set(0);
    // }
    // else{
    // Roller.set(.5);
    // }

    // return MotorFinish;
    // }
}
