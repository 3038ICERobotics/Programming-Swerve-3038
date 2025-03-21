package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {

    // Initialize Motors [ EDIT ]
    SparkMax ElevatorLeft = new SparkMax(MotorIDs.ElevatorLeftID, MotorType.kBrushless);
    SparkMax ElevatorRight = new SparkMax(MotorIDs.ElevatorRightID, MotorType.kBrushless);
    SparkMax AlgaeBooter = new SparkMax(MotorIDs.AlgaeBooterID, MotorType.kBrushless);
    SparkFlex OuttakeRoller = new SparkFlex(MotorIDs.OuttakeRollerID, MotorType.kBrushless);
    DigitalInput BreakBeamClear = new DigitalInput(6);
    DigitalInput BreakBeamCoral = new DigitalInput(2);
    public ClosedLoopConfig ElevatorLoopConfig = new ClosedLoopConfig();
    public SparkBaseConfig ElevatorBaseConfig = new SparkMaxConfig();
    SparkMaxConfig ElevatorConfig = new SparkMaxConfig();
    SparkClosedLoopController ElevatorPIDRight;
    SparkClosedLoopController ElevatorPIDLeft;
    RelativeEncoder AlgaeEncoder;
    public RelativeEncoder RightElevatorEncoder;
    public RelativeEncoder LeftElevatorEncoder;

    public Elevator(ClosedLoopConfig config) {
        //ElevatorConfig.follow(ElevatorRight, false);
        ElevatorPIDRight = ElevatorRight.getClosedLoopController();
        ElevatorPIDLeft = ElevatorLeft.getClosedLoopController();
        ElevatorBaseConfig = new SparkMaxConfig();
        ElevatorBaseConfig.apply(config);
        ElevatorRight.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        ElevatorLeft.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlgaeEncoder = AlgaeBooter.getEncoder();
        AlgaeEncoder.setPosition(0);
        RightElevatorEncoder  = ElevatorRight.getEncoder();
        LeftElevatorEncoder  = ElevatorLeft.getEncoder();
        RightElevatorEncoder.setPosition(0);
        LeftElevatorEncoder.setPosition(0);
    }

    public void DisplayPosition(){
        SmartDashboard.putNumber("ElevatorEncoderRight", RightElevatorEncoder.getPosition());
        SmartDashboard.putNumber("ElevatorEncoderLeft", LeftElevatorEncoder.getPosition());
        SmartDashboard.putNumber("AlgaeBooter Encoder", AlgaeEncoder.getPosition());
    }

    public void UpdatePID(ClosedLoopConfig config) {
        ElevatorBaseConfig.apply(config);
        ElevatorRight.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        ElevatorLeft.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Positions
    public enum ElevatorPositions {
        Home, Player, Tray, First, Second, Third
    }

    float HomePosition = 0;
    float PlayerPosition = 1;
    float TrayPosition = 2;
    float FirstPosition = 3;
    float SecondPosition = 4;
    float ThirdPosition = 5;
    float[] HeightRotations = { HomePosition, PlayerPosition, TrayPosition, FirstPosition, SecondPosition,
            ThirdPosition };

    public boolean GoToHeight(int TargetPosition) {
        double HeightSetPoint;
        boolean Result = false;
        HeightSetPoint = HeightRotations[TargetPosition];
        SmartDashboard.putNumber("HeightSetPoint", HeightSetPoint);
       // ElevatorPID.setReference(HeightSetPoint, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

        if (RightElevatorEncoder.getPosition() == HeightSetPoint) {
            Result = true;
        }
        // if (ElevatorPositions.Home.getOrdinal()) { run motors until encoders reach 0
        // }
        //
        return Result;
        // Elevator goes to the desired height.
        // True if elevator is at the correct position; false otherwise.
    }
    public void Test (){
        double ElevatorSetpoint =  SmartDashboard.getNumber("Elevator Setpoint", 0);
        ElevatorPIDLeft.setReference(ElevatorSetpoint, ControlType.kPosition);
        ElevatorPIDRight.setReference(ElevatorSetpoint, ControlType.kPosition);
        SmartDashboard.putNumber("Elevator Setpoint", ElevatorSetpoint);
    }

    public boolean ScoreCoral() {
        boolean Coral = BreakBeamCoral.get();
        if (Coral) {
            OuttakeRoller.set(0);
        } else {
            OuttakeRoller.set(0.5);
        }
        return Coral;
    }

    public boolean IntakeCoral() {
        boolean HasCoral = BreakBeamClear.get() && !BreakBeamCoral.get();
        if (HasCoral) {
            OuttakeRoller.set(0);
        } else {
            OuttakeRoller.set(0.5);
        }
        return HasCoral;
    }

    public void BreakBeamSignal() {
        SmartDashboard.putBoolean("BreakBeamClear", BreakBeamClear.get());
    }
}
