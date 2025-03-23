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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {

    // Initialize Motors [ EDIT ]

    /* Elevator Lift */
    SparkMax ElevatorLeft = new SparkMax(MotorIDs.ElevatorLeftID, MotorType.kBrushless);
    SparkMax ElevatorRight = new SparkMax(MotorIDs.ElevatorRightID, MotorType.kBrushless);
    public ClosedLoopConfig ElevatorLoopConfig = new ClosedLoopConfig();
    public SparkBaseConfig ElevatorBaseConfig = new SparkMaxConfig();
    SparkMaxConfig ElevatorConfig = new SparkMaxConfig();
    SparkClosedLoopController ElevatorPIDRight;
    SparkClosedLoopController ElevatorPIDLeft;
    public RelativeEncoder RightElevatorEncoder;
    public RelativeEncoder LeftElevatorEncoder;

    /* Algae Booter */
    SparkMax AlgaeBooter = new SparkMax(MotorIDs.AlgaeBooterID, MotorType.kBrushless);
    public ClosedLoopConfig AlgaeBooterPIDConfig = new ClosedLoopConfig();
    public SparkBaseConfig AlgaeBooterBaseConfig = new SparkMaxConfig();
    SparkClosedLoopController AlgaeBooterPID;
    RelativeEncoder AlgaeEncoder;
    boolean IsBooterOut = false;

    /* Coral Outtake */
    SparkFlex OuttakeRoller = new SparkFlex(MotorIDs.OuttakeRollerID, MotorType.kBrushless);
    DigitalInput BreakBeamClear = new DigitalInput(6);
    boolean previous = true;
    // DigitalInput BreakBeamCoral = new DigitalInput(2);

    public Elevator(ClosedLoopConfig config) {
        /* Elevator Lift */
        // ElevatorConfig.follow(ElevatorRight, false);
        ElevatorPIDRight = ElevatorRight.getClosedLoopController();
        ElevatorPIDLeft = ElevatorLeft.getClosedLoopController();
        ElevatorBaseConfig = new SparkMaxConfig();
        ElevatorBaseConfig.apply(config);
        ElevatorRight.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        ElevatorLeft.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        RightElevatorEncoder = ElevatorRight.getEncoder();
        LeftElevatorEncoder = ElevatorLeft.getEncoder();
        RightElevatorEncoder.setPosition(0);
        LeftElevatorEncoder.setPosition(0);


        /* Algae */
        AlgaeEncoder = AlgaeBooter.getEncoder();
        AlgaeEncoder.setPosition(0);
        AlgaeBooterPIDConfig.pidf(1,0,0.05,0);
        AlgaeBooterBaseConfig.apply(AlgaeBooterPIDConfig);
        AlgaeBooterBaseConfig.idleMode(IdleMode.kBrake);
        AlgaeBooterBaseConfig.smartCurrentLimit(20); // Default limit is 80A - this limit is too high for a NEO 550
        AlgaeBooter.configure(AlgaeBooterBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlgaeBooterPID = AlgaeBooter.getClosedLoopController();
        
    }

    public void DisplayPosition() {
        SmartDashboard.putNumber("ElevatorEncoderRight", RightElevatorEncoder.getPosition());
        SmartDashboard.putNumber("ElevatorEncoderLeft", LeftElevatorEncoder.getPosition());
        SmartDashboard.putNumber("AlgaeBooter Encoder", AlgaeEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Target", CurrentTargetSetpoint);
        BreakBeamSignal();
    }

    public void UpdatePID(ClosedLoopConfig config) {
        ElevatorBaseConfig.apply(config);
        ElevatorRight.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        ElevatorLeft.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Positions
    public enum ElevatorPositions {
        Home, Load, Tray, First, Second
    }

    double CurrentTargetSetpoint = 0;
    float HomePosition = 0;
    float LoadPosition = -10;
    float TrayPosition = -10;
    float FirstPosition = -20;
    float SecondPosition = -30;
    // float ThirdPosition = 5;
    float[] HeightRotations = { HomePosition, LoadPosition, TrayPosition, FirstPosition, SecondPosition,
    };

    //return true to continue, false to stop
    public boolean GoToHeight(int TargetPosition) {
        boolean Result = false;
        CurrentTargetSetpoint = HeightRotations[TargetPosition];
        SmartDashboard.putNumber("HeightSetPoint", CurrentTargetSetpoint);
        ElevatorPIDLeft.setReference(CurrentTargetSetpoint, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        ElevatorPIDRight.setReference(CurrentTargetSetpoint, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

        if (RightElevatorEncoder.getPosition() == CurrentTargetSetpoint) {
            Result = true;
        }
        // if (ElevatorPositions.Home.getOrdinal()) { run motors until encoders reach 0
        // }
        //
        return false; // Always return false - else we will never be at exact setpoint
        // Elevator goes to the desired height.
        // True if elevator is at the correct position; false otherwise.
    }

    public void FineAdjustment(double deltaPosition) {
        // double ElevatorSetpoint = SmartDashboard.getNumber("Elevator Setpoint", 0);
        CurrentTargetSetpoint += deltaPosition;
        if(CurrentTargetSetpoint>0){
            CurrentTargetSetpoint=0;
        }
        if(CurrentTargetSetpoint<-30){
            CurrentTargetSetpoint= -30;
        }
        ElevatorPIDLeft.setReference(CurrentTargetSetpoint, ControlType.kPosition);
        ElevatorPIDRight.setReference(CurrentTargetSetpoint, ControlType.kPosition);
        //SmartDashboard.putNumber("Elevator Setpoint", ElevatorSetpoint);
    }

    public boolean ScoreCoral(boolean Coral) {
        if (Coral) {
            OuttakeRoller.set(-0.8);
        } else {
            OuttakeRoller.set(0);
        }
        return Coral;
    }

    //return true to continue, false to stop
    public boolean IntakeCoral() {
        boolean HasCoral = BreakBeamClear.get();
        boolean result = !(!previous && HasCoral);//false if previous was false and hascoral is true
        previous = HasCoral;
        if (result) {
            OuttakeRoller.set(-0.05);
        } else {
            OuttakeRoller.set(0);
        }
        return result;
    }

    public void BreakBeamSignal() {
        SmartDashboard.putBoolean("BreakBeamClear", BreakBeamClear.get());
    }
    //return true to continue, false to stop
    public boolean ToggleBooter() {
        if (IsBooterOut == true){
           AlgaeBooterPID.setReference(-1,ControlType.kPosition);
        }
        else {
           AlgaeBooterPID.setReference(-2.3,ControlType.kPosition);
        }
        IsBooterOut = !IsBooterOut;
        return false;
    }
}
