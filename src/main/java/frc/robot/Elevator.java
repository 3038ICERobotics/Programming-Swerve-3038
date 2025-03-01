package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.SparkMax;

public class Elevator {

//Initialize Motors [ EDIT ]
SparkMax ElevatorLeft = new SparkMax(0, MotorType.kBrushless);
SparkMax ElevatorRight = new SparkMax(0, MotorType.kBrushless);
SparkMax AlgaeBooter = new SparkMax(0, MotorType.kBrushless);
SparkFlex OuttakeRoller = new SparkFlex(0, MotorType.kBrushless);
RelativeEncoder LeftEncoder;
RelativeEncoder RightEncoder;
DigitalInput BreakBeamClear = new DigitalInput(1);
DigitalInput BreakBeamCoral = new DigitalInput(2);

//Positions
public enum ElevatorPositions{Home, Player, Tray, First, Second, Third}
public boolean GoToTrayHeight(ElevatorPositions TargetPosition) {

return false;
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
