package frc.robot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
        SparkMax Climber = new SparkMax(MotorIDs.HookID, MotorType.kBrushless);
        RelativeEncoder ClimberEncoder = Climber.getEncoder();
        SparkMaxConfig LeftConfig = new SparkMaxConfig();
        SparkClosedLoopController PID;
        SparkBaseConfig BaseConfig;
        double TargetPosition = 0;
        double ExtendSetpoint = -70;
        double RetractSetpoint = 12;

        public Climber(ClosedLoopConfig config) {
                PID = Climber.getClosedLoopController();
                BaseConfig = new SparkMaxConfig();
                BaseConfig.apply(config);
                ClimberEncoder.setPosition(0);
                Climber.configure(BaseConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                SmartDashboard.putNumber("ClimberSetpoint", 0);
        }

        public void DisplayPosition() {
                SmartDashboard.putNumber("ClimberEncoder", ClimberEncoder.getPosition());
        }

        public void ExtendClimber() {
                // double HookSetpoint = SmartDashboard.getNumber("Hook Setpoint", 0);
                PID.setReference(ExtendSetpoint, ControlType.kPosition);
                SmartDashboard.putNumber("Hook Setpoint", ExtendSetpoint);
        }

        public void RetractClimber() {
                PID.setReference(RetractSetpoint, ControlType.kPosition);
                SmartDashboard.putNumber("Hook Setpoint", RetractSetpoint);
        }

        public void Test() {
                double val = SmartDashboard.getNumber("ClimberSetpoint", 0);
                PID.setReference(val, ControlType.kPosition);
        }
}
