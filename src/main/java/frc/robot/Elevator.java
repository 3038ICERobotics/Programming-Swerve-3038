package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Elevator {

//Initialize Motors [ EDIT ]
SparkMax ElevatorLeft = new SparkMax(0, MotorType.kBrushless);
SparkMax ElevatorRight = new SparkMax(0, MotorType.kBrushless);
SparkMax AlgaeBooter = new SparkMax(0, MotorType.kBrushless);
SparkFlex OuttakeRoller = new SparkFlex(0, MotorType.kBrushless);


}
