package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AnalogContainer;

public class SwerveDrive {

    public final double MaxDriveSpeed = .1;
    public final double MaxTurnSpeed = .1;

    public enum ModuleOrder {
        FL, BL, FR, BR
    };

    public SparkMax[] driveMotors = {
            new SparkMax(10, MotorType.kBrushless), // FrontLeftDrive
            new SparkMax(22, MotorType.kBrushless), // BackLeftDrive
            new SparkMax(3, MotorType.kBrushless), // FrontRightDrive
            new SparkMax(14, MotorType.kBrushless),// BackRightDrive
    };

    // public TalonFX[] driveMotorsTalon = {
    // new TalonFX(10),
    // new TalonFX(22),
    // new TalonFX(3),
    // new TalonFX(14),
    // };

    public SparkMax[] steerMotors = {
            new SparkMax(15, MotorType.kBrushless), // FrontLeftSteer
            new SparkMax(8, MotorType.kBrushless), // BackLeftSteer
            new SparkMax(1, MotorType.kBrushless), // FrontRightSteer
            new SparkMax(2, MotorType.kBrushless), // BackRightSteer
    };
    public SparkClosedLoopController[] drivePIDControllers = new SparkClosedLoopController[4];
    public SparkClosedLoopController[] steerPIDControllers = new SparkClosedLoopController[4];
    public RelativeEncoder[] driveEncoders = new RelativeEncoder[4];
    public RelativeEncoder[] steerEncoders = new RelativeEncoder[4];
    public AnalogContainer[] analogEncoders = new AnalogContainer[4];

    private ClosedLoopConfig VelocityLoopConfig = new ClosedLoopConfig();
    private ClosedLoopConfig SteeringLoopConfig = new ClosedLoopConfig();
    private SparkBaseConfig SteeringBaseConfig = new SparkMaxConfig();
    private SparkBaseConfig VelocityBaseConfig = new SparkMaxConfig();
    /* 
     * Start at velocity 0, use slot 0 
    */
    // private final VelocityVoltage krackenVelocity = new VelocityVoltage(0).withSlot(0);
    /*
     * Keep a neutral out so we can disable the motor 
    */
    // private final NeutralOut m_brake = new NeutralOut();

    SwerveModuleState[] moduleStates;

    private final int RotationsPerMeter = 27;
    private final int SecondsPerMinute = 60;
    private final double GearRation = 55;
    private final double DegreeToRadians = Math.PI / 180;

    private SwerveDriveKinematics Kinematics = new SwerveDriveKinematics(new Translation2d(0.285, 0.285),
            new Translation2d(-0.285, 0.285),
            new Translation2d(0.285, -0.285), new Translation2d(-0.285, -0.285));

    public SwerveDrive() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        /*
         * Voltage-based velocity requires a velocity feed forward to account for the
         * back-emf of the motor
         *
         * To account for friction, add 0.1 V of static feedforward
         * Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
         * volts / rotation per second
         * An error of 1 rotation per second results in 0.11 V output
         * No output for integrated error
         * No output for error derivative
         * Peak output of 8 volts
        */
        configs.Slot0.kS = 0.1;
        configs.Slot0.kV = 0.12;
        configs.Slot0.kP = 0.11;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 0;

        configs.Voltage.withPeakForwardVoltage(Volts.of(8))
                .withPeakReverseVoltage(Volts.of(-8));

        for (int i = 0; i < 4; i++) {
            drivePIDControllers[i] = driveMotors[i].getClosedLoopController();
            /* 
             * Retry config apply up to 5 times, report if failure 
            */
            // StatusCode status = StatusCode.StatusCodeNotInitialized;
            // for (int j = 0; j < 5; ++j) {
            // status = driveMotorsTalon[i].getConfigurator().apply(configs);
            // if (status.isOK())
            // break;
            // }
            // if (!status.isOK()) {
            // System.out.println("Could not apply configs, error code: " +
            // status.toString());
            // }
            driveEncoders[i] = driveMotors[i].getEncoder();
            driveEncoders[i].setPosition(0);
            steerPIDControllers[i] = steerMotors[i].getClosedLoopController();
            steerEncoders[i] = steerMotors[i + 4].getEncoder();
            steerEncoders[i].setPosition(0);
        }
        analogEncoders[0] = new AnalogContainer(steerMotors[0].getAnalog(), 2.23, 1.60);
        analogEncoders[1] = new AnalogContainer(steerMotors[1].getAnalog(), 2.23, 2.18);
        analogEncoders[2] = new AnalogContainer(steerMotors[2].getAnalog(), 2.27, 1.78);
        analogEncoders[3] = new AnalogContainer(steerMotors[3].getAnalog(), 2.20, 0.12);

        VelocityLoopConfig.pidf(0.000170, 0.000001, 0.000020, 0.000001, ClosedLoopSlot.kSlot0);
        VelocityLoopConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        SteeringLoopConfig.pidf(1, 0.5, 0.1, 0.00001, ClosedLoopSlot.kSlot0);
        SteeringLoopConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        VelocityBaseConfig.apply(VelocityLoopConfig);
        VelocityBaseConfig.inverted(false);

        SteeringBaseConfig.apply(SteeringLoopConfig);
        SteeringBaseConfig.inverted(false);

        for (int i = 0; i < 4; i++) {
            driveMotors[i].configure(VelocityBaseConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
            steerMotors[i].configure(SteeringBaseConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }
    }

    public void AnalogInit() {
        for (int i = 0; i < 4; i++) {
            analogEncoders[i].offset = analogEncoders[i].getRotation();
            steerEncoders[i].setPosition(analogEncoders[i].offset * -GearRation);
        }
    }

    public void driveRobotOriented(double distX, double distY, double rotRad){
        drive(distX,distY,rotRad);
    }

    public void driveFieldOriented(double distX, double distY, double rotRad){
        /* 
         * perform vector rotation of distX,distY by the negative angle
         * from the current heading of the gyro
         */
        double[]vec = rotateByAngle(new double[]{distX,distY}, 0.0);//pass -gyro.yaw instead of 0.
        drive(vec[0],vec[1],rotRad);
    }

    private double[] rotateByAngle(double[] vec, double rad){
        return new double[]{
            Math.cos(rad)*vec[0]-Math.sin(rad)*vec[1],
            Math.sin(rad)*vec[0]+Math.cos(rad)*vec[1],
        };
    }

    private void drive(double distX, double distY, double rotRad) {
        performKinematicWith(new ChassisSpeeds(distX, distY, rotRad));

        for (int i = 0; i < 4; i++) {
            drivePIDControllers[i]
                    .setReference(moduleStates[i].speedMetersPerSecond * RotationsPerMeter * SecondsPerMinute,
                            SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.00001);
            // driveMotorsTalon[i].setControl(
            // krackenVelocity.withVelocity(OptimizedStates[i].speedMetersPerSecond *
            // RotationsPerMeter * SecondsPerMinute));
            /*
             * Add in the current steerEncoder position because the moduleState
             * has the delta angle instead of the absolute angle.
             */
            steerPIDControllers[i]
                    .setReference((moduleStates[i].angle.getRotations() * GearRation) + steerEncoders[i].getPosition(),
                            SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.00001);
        }
    }

    private void performKinematicWith(ChassisSpeeds speeds) {
        moduleStates = Kinematics.toSwerveModuleStates(speeds);
        angleMinimize(moduleStates);
        Kinematics.desaturateWheelSpeeds(moduleStates, MaxDriveSpeed);
    }

    private void angleMinimize(SwerveModuleState[] mStates) {
        double deltaAngle, direction;
        for (int i = 0; i < 4; i++) {
            deltaAngle = mStates[i].angle.getDegrees() - analogEncoders[i].getDegrees();
            direction = deltaAngle > 0 ? 1 : -1;
            if (360 - Math.abs(deltaAngle) < Math.abs(deltaAngle)) {
                /*
                 * This means that it is a shorter distance to rotate in
                 * the opposite direction, for the complement of the angle.
                 */
                deltaAngle = (360 - Math.abs(deltaAngle)) * (-direction);
            }
            /*
             * since we are storing the delta angle in mState
             * instead of the absolute angle we will need to add it to the
             * current steerEncoder.position before the setreference.
             */
            mStates[i].angle = new Rotation2d(deltaAngle * DegreeToRadians);
        }
    }
}
