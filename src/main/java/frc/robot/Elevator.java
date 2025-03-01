package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

//Initialize Motors [ EDIT ]
SparkMax ElevatorLeft = new SparkMax(99, MotorType.kBrushless);
SparkMax ElevatorRight = new SparkMax(98, MotorType.kBrushless);
SparkMax AlgaeBooter = new SparkMax(7, MotorType.kBrushless);
SparkFlex OuttakeRoller = new SparkFlex(3, MotorType.kBrushless);
RelativeEncoder LeftEncoder;
RelativeEncoder RightEncoder;
DigitalInput BreakBeamClear = new DigitalInput(1);
DigitalInput BreakBeamCoral = new DigitalInput(2);
public ClosedLoopConfig ElevatorLoopConfig = new ClosedLoopConfig();
public SparkBaseConfig ElevatorBaseConfig = new SparkMaxConfig();
SparkMaxConfig ElevatorConfig = new SparkMaxConfig();
SparkClosedLoopController ElevatorPID;

public Elevator() {
    ClosedLoopConfig config = new ClosedLoopConfig();
    ElevatorConfig.follow(ElevatorRight, true);
    ElevatorPID = ElevatorRight.getClosedLoopController();
    ElevatorBaseConfig = new SparkMaxConfig();
    ElevatorBaseConfig.apply(config);
    ElevatorRight.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    ElevatorLeft.configure(ElevatorBaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
}

public RelativeEncoder RightElevatorEncoder = ElevatorRight.getEncoder();
//Positions
public enum ElevatorPositions{Home, Player, Tray, First, Second, Third}
float HomePosition = 0;
float PlayerPosition = 1;
float TrayPosition = 2;
float FirstPosition = 3;
float SecondPosition = 4;
float ThirdPosition = 5;
float[] HeightRotations = {HomePosition, PlayerPosition, TrayPosition, FirstPosition, SecondPosition, ThirdPosition};

public boolean GoToHeight(int TargetPosition) {
    double HeightSetPoint;
    boolean Result = false;
    HeightSetPoint = HeightRotations[TargetPosition];
    SmartDashboard.putNumber("HeightSetPoint", HeightSetPoint);
    ElevatorPID.setReference(HeightSetPoint, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

    if (RightElevatorEncoder.getPosition() == HeightSetPoint) {
        Result = true;
    }
    // if (ElevatorPositions.Home.getOrdinal()) { run motors until encoders reach 0 }
    //
return Result;
//Elevator goes to the desired height.
//True if elevator is at the correct position; false otherwise.
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

}
