package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorConfigs {
    // Static utility class for configuring hopper motors

    // PID configurator
    public static void configurePID(
        TalonFX[] hopperMotors, 
        double p, double i, double d,
        double s, double v, double a,
        double cruiseVel, double accel, double jerk) {
        // PID configurator for the hopper motor
        for (int n = 0; n < hopperMotors.length; n++) {
            var hopperConfig = new TalonFXConfiguration();
            hopperMotors[n].getConfigurator().refresh(hopperConfig);
            var hopperSlot0Configs = hopperConfig.Slot0;
            hopperSlot0Configs.kP = p;
            hopperSlot0Configs.kI = i;
            hopperSlot0Configs.kD = d;
            hopperSlot0Configs.kS = s;
            hopperSlot0Configs.kV = v;
            hopperSlot0Configs.kA = a;

            // set Motion Magic settings
            var motionMagicConfigs = hopperConfig.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVel; // Target cruise velocity
            motionMagicConfigs.MotionMagicAcceleration = accel; // Target acceleration
            motionMagicConfigs.MotionMagicJerk = jerk; // Target jerk for smooth S-Curve

            // Apply entire config INCLUDING Motion Magic settings
            hopperMotors[n].getConfigurator().apply(hopperConfig);
        }
    }

    public static void configureMotors(TalonFX[] hopperMotors) {
        // They are grouped together, but they are treated independently
        TalonFXConfiguration[] hopperConfig = new TalonFXConfiguration[] {
            new TalonFXConfiguration(),
            new TalonFXConfiguration()
        };

        // Set current limits for hopper motors
        for (TalonFXConfiguration config : hopperConfig) {
            config.CurrentLimits.SupplyCurrentLimit = 25;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }
    
        // Set voltage limits for hopper motors
        for (TalonFXConfiguration config : hopperConfig) {
            config.Voltage.PeakForwardVoltage = HopperConstants.maxVoltage;
            config.Voltage.PeakReverseVoltage = -HopperConstants.maxVoltage;
        }

        // Set the motors to brake on during idle
        for (TalonFXConfiguration config : hopperConfig) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }

        // Set the directions for the motors
        hopperConfig[0].MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig[1].MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure PID for the hopper motors
        configurePID(
            hopperMotors,
            HopperConstants.kPID[0], // Kp
            HopperConstants.kPID[1], // kI
            HopperConstants.kPID[2], // kD
            HopperConstants.kPID[3], // kS
            HopperConstants.kPID[4], // kV
            HopperConstants.kPID[5], // kA
            HopperConstants.kPID[6], // Motion Magic cruise velocity
            HopperConstants.kPID[7], // Motion Magic acceleration
            HopperConstants.kPID[8]  // Motion Magic jerk
        );

        // Apply hopper motor configs
        for (int i = 0; i < hopperMotors.length; i++) {
            hopperMotors[i].getConfigurator().apply(hopperConfig[i]);
        }

        System.out.println("Hopper master ID: " + hopperMotors[0].getDeviceID());
        System.out.println("Hopper follower ID: " + hopperMotors[1].getDeviceID());

        System.out.println("Hopper motor configs applied!");
    }
}
