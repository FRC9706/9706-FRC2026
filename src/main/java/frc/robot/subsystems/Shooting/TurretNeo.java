package frc.robot.subsystems.Shooting;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class TurretNeo extends SubsystemBase {
    // Create an instance for the TurretNeo
    public static TurretNeo mInstance = null;
    public static TurretNeo getInstance() {
        if (mInstance == null) {
            mInstance = new TurretNeo();
        }
        return mInstance;
    }

    // Initialize the motors for the turret
    private SparkMax rotationMotor;
        // Intialize stuff for the rotation motor
        private SparkClosedLoopController rotCL;
        private RelativeEncoder rotEN;

    // Initialize Can Coders for the turret motors
    private CANcoder pitchCC;

    // Initalize limelight variables
    private double tx;
    private double ty;
    private boolean tv;

    // Initalize turret variables
    private double turretAngleDeg;   // Current turret angle

    public static enum state {
        Idle,
        roamPos,
        roamNeg,
        Tracking,
    }

    private state currentState = state.Idle;

    public state getState() {
        return currentState;
    }

    public void setState(state newState) {
        currentState = newState;
    }

    public void configureMotors() {
        // Rotional motor configs
        var rotConfig = new SparkMaxConfig();

        // Configuration for slot 0 configs for the rotational motor
        var rotSlot0Configs = rotConfig;
        rotSlot0Configs.inverted(false);
        rotSlot0Configs.idleMode(IdleMode.kCoast);
        rotSlot0Configs.smartCurrentLimit(10);

        rotSlot0Configs.closedLoop 
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Constants.Turret.kRotPID[0])
        .i(Constants.Turret.kRotPID[1])
        .d(Constants.Turret.kRotPID[2])
        .outputRange(-1, 1);

        // Apply the configurations to the motor
        rotationMotor.configure(rotSlot0Configs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        System.out.println("Motor configurations applied!");
    }

    public TurretNeo() {
        // Create the motor objects
        rotationMotor = new SparkMax(Constants.Turret.rotationMotor, MotorType.kBrushless);
            // Create the objects also needed for the rotation motor
            rotCL = rotationMotor.getClosedLoopController();
            rotEN = rotationMotor.getEncoder();
        

        // Apply the configurations for the motors
        configureMotors();

        // Create the CANcoder objects
        pitchCC = new CANcoder(Constants.Turret.pitchCanCoder);
    }


    public void stopRotationMotor() {
        rotationMotor.stopMotor();
    }

    public void startAutoTrack() {
        currentState = state.roamPos;
    }

    public void rotate(double power) {
        power = Math.copySign((Math.min(Math.abs(power), Constants.Turret.maxRotPower)), power);

        if ((turretAngleDeg >= Constants.Turret.turretDegLim) && power > 0 
                || (turretAngleDeg <= -Constants.Turret.turretDegLim && power < 0)) {
                    stopRotationMotor();
                    return;
        }
    
        rotationMotor.set(power);
    }

    public void targetTracking() {
        // No targets -> back to TagFinding
        if (!tv) {
            if (turretAngleDeg >= 0) {
                currentState = state.roamNeg;
            } else if (turretAngleDeg < 0) {
                currentState = state.roamPos;
            }
        }

        // Deadband to prevent motor overshoot
        if (Math.abs(tx) < Constants.Turret.txDeadbandDeg) {
            stopRotationMotor();
            return;
        }

        // Simple proportional control
        double rotationPower = tx * Constants.Turret.kRotPID[0];

        // Clamp output power
        rotationPower = Math.copySign(
            Math.min(Math.abs(rotationPower), Constants.Turret.maxRotPower),
            rotationPower
        );

        rotate(rotationPower);
    }

    public void getDistance() {
        // For future distance calculations
    }

    @Override
    public void periodic() {
        // Asign limelight variables periodically to update continously
        tx = LimelightHelpers.getTX("turretLimelight");
        ty = LimelightHelpers.getTY("turretLimelight");
        tv = LimelightHelpers.getTV("turretLimelight");

        // Asign turret rotational values for calculations
        turretAngleDeg = rotEN.getPosition();

        // STATE LOGIC
        switch (currentState) {
            case Idle:
                stopRotationMotor();
                break;

            case roamPos:
                if (tv) {
                    currentState = state.Tracking;
                } else if (turretAngleDeg >= Constants.Turret.turretDegLim) {
                    currentState = state.roamNeg;
                } else if (turretAngleDeg < Constants.Turret.turretDegLim) {
                    rotate(Constants.Turret.roamSpeed);
                }
                break;

            case roamNeg:
                if (tv) {
                    currentState = state.Tracking;
                } else if (turretAngleDeg <= -Constants.Turret.turretDegLim) {
                    currentState = state.roamPos;
                } else if (turretAngleDeg > -Constants.Turret.turretDegLim) {
                    rotate(-Constants.Turret.roamSpeed);
                }
                break;

            case Tracking:
                targetTracking();
                break;
        }
    }
}
