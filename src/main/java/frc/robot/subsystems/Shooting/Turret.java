package frc.robot.subsystems.Shooting;

import java.lang.Thread.State;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class Turret extends SubsystemBase{
    // Create an instance for the Turret
    public static Turret mInstance = null;
    public static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }
        return mInstance;
    }

    // Initialize the motors for the turret
    private TalonFX firingMotor;
    private TalonFX rotationMotor;
    private TalonFX pitchMotor;

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
        var rotConfig = new TalonFXConfiguration();

        // Pitch motor configs
        var pitchConfig = new TalonFXConfiguration();

        // Configuration for slot 0 configs for the rotational motor
        var rotSlot0Configs = rotConfig.Slot0;
        rotSlot0Configs.kP = Constants.Turret.kRotPID[0];
        rotSlot0Configs.kI = Constants.Turret.kRotPID[1];
        rotSlot0Configs.kD = Constants.Turret.kRotPID[2];

        // Configuration for slot 0 configs for the pitch motor
        var pitchSlot0Configs = pitchConfig.Slot0;
        pitchSlot0Configs.kP = Constants.Turret.kPitchPID[0];
        pitchSlot0Configs.kI = Constants.Turret.kPitchPID[1];
        pitchSlot0Configs.kD = Constants.Turret.kPitchPID[2];

        // Apply the configurations to each motor respectivly
        rotationMotor.getConfigurator().apply(rotSlot0Configs);
        pitchMotor.getConfigurator().apply(pitchSlot0Configs);

        System.out.println("Motor configurations applied!");
    }

    public Turret() {
        // Create the motor objects
        firingMotor = new TalonFX(Constants.Turret.firingMotor);
        rotationMotor = new TalonFX(Constants.Turret.rotationMotor);
        pitchMotor = new TalonFX(Constants.Turret.pitchMotor);

        // Apply the configurations for the motors
        configureMotors();

        // Make the motors go to zero
        rotationMotor.setControl(new PositionDutyCycle(0));
        pitchMotor.setControl(new PositionDutyCycle(0));

        // Create the CANcoder objects
        pitchCC = new CANcoder(Constants.Turret.pitchCanCoder);
    }

    public void stopFiringMotor() {
        firingMotor.stopMotor();
    }

    public void stopRotationMotor() {
        rotationMotor.stopMotor();
    }

    public void stopTiltMotor() {
        pitchMotor.stopMotor();
    }

    public void rotate(double power) {
        power = Math.copySign((Math.min(Math.abs(power), Constants.Turret.maxRotPower)), power);

        if ((turretAngleDeg >= Constants.Turret.turretDegLim) && power > 0 
                || (turretAngleDeg <= -Constants.Turret.turretDegLim && power < 0)) {
                    stopRotationMotor();
                    return;
        }
    
        rotationMotor.setControl(new DutyCycleOut(power));
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
        turretAngleDeg = Math.toDegrees(rotationMotor.getPosition().getValueAsDouble()) * 360;

        // STATE LOGIC
        switch (currentState) {
            case Idle:
                stopFiringMotor();
                stopRotationMotor();
                stopTiltMotor();
                break;

            case roamPos:
                if (!tv) {
                    currentState = state.Tracking;
                } else if (turretAngleDeg >= Constants.Turret.turretDegLim) {
                    currentState = state.roamNeg;
                } else if (turretAngleDeg < Constants.Turret.turretDegLim) {
                    rotate(Constants.Turret.roamSpeed);
                }
                break;

            case roamNeg:
                if (!tv) {
                    currentState = state.Tracking;
                } else if (turretAngleDeg <= -Constants.Turret.turretDegLim) {
                    currentState = state.roamPos;
                } else if (turretAngleDeg > -Constants.Turret.turretDegLim) {
                    rotate(-Constants.Turret.roamSpeed);
                }

            case Tracking:
                targetTracking();
                break;
        }
    }
}
