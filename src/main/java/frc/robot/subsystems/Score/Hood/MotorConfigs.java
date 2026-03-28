package frc.robot.subsystems.Score.Hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


public class MotorConfigs {
    public static final String DynamicInputs = null;

    // Static utility class for configuring the hood motor
    public static void configureHoodPID(
        TalonFX hoodMotor, 
        double p, double i, double d,
        double s, double v, double a,
        double cruiseVel, double accel, double jerk) {
        // PID configurator for the rotational motor
        var hoodConfig = new TalonFXConfiguration();
        hoodMotor.getConfigurator().refresh(hoodConfig);
        var rotSlot0Configs = hoodConfig.Slot0;
        rotSlot0Configs.kP = p;
        rotSlot0Configs.kI = i;
        rotSlot0Configs.kD = d;
        rotSlot0Configs.kS = s;
        rotSlot0Configs.kV = v;
        rotSlot0Configs.kA = a;

        // set Motion Magic settings
        var motionMagicConfigs = hoodConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVel; // Target cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = accel; // Target acceleration
        motionMagicConfigs.MotionMagicJerk = jerk; // Target jerk for smooth S-Curve

        // Apply entire config INCLUDING Motion Magic settings
        hoodMotor.getConfigurator().apply(hoodConfig);
    }

    public static void configurHoodMotor(TalonFX hoppMotor) {
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

        // Set current limits for all motors
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.Voltage.PeakForwardVoltage = HoodConstants.maxVoltage;
        hoodConfig.Voltage.PeakReverseVoltage = -HoodConstants.maxVoltage;

        // set state of the rotation motor
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // set MagicMotion PID for rotation motor
        configureHoodPID(
            hoppMotor,
            HoodConstants.hoodPID[0], // Kp
            HoodConstants.hoodPID[1], // kI
            HoodConstants.hoodPID[2], // kD
            HoodConstants.hoodPID[3], // kS
            HoodConstants.hoodPID[4], // kV
            HoodConstants.hoodPID[5], // kA
            HoodConstants.hoodPID[6], // Motion Magic cruise velocity
            HoodConstants.hoodPID[7], // Motion Magic acceleration
            HoodConstants.hoodPID[8]  // Motion Magic jerk
        );

        // Apply motor configs
        hoppMotor.getConfigurator().apply(hoodConfig);

        System.out.println("Hood motor configs applied!");
    }    
}
