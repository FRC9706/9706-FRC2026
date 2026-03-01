package frc.robot.subsystems.Score.TurretModules;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.Score.TurretConstants;

public class MotorConfigs {
    // Static utility class for configuring turret motors

    public static void configureRotPID(TalonFX rotMotor, double p, double i, double d) {
        // PID configurator for the rotational motor
        var rotConfig = new TalonFXConfiguration();
        rotMotor.getConfigurator().refresh(rotConfig);
        var rotSlot0Configs = rotConfig.Slot0;
        rotSlot0Configs.kP = p;
        rotSlot0Configs.kI = i;
        rotSlot0Configs.kD = d;

        rotMotor.getConfigurator().apply(rotSlot0Configs);
    }

    public static void configureFirePID(TalonFX[] shootMotors, double p, double i, double d) {
        // PID configurator for the firing motors (assumes both have same PID)
        for (int n = 0; n < shootMotors.length; n++) {
            var fireConfig = new TalonFXConfiguration();
            shootMotors[n].getConfigurator().refresh(fireConfig);
            var slot0Configs = fireConfig.Slot0;
            slot0Configs.kP = p;
            slot0Configs.kI = i;
            slot0Configs.kD = d;
            slot0Configs.kV = 0.12;
            slot0Configs.kS = 0.25;
            shootMotors[n].getConfigurator().apply(fireConfig);
        }
    }

    public static void configureMotors(TalonFX rotMotor, TalonFX[] shootMotors) {
        TalonFXConfiguration rotConfig = new TalonFXConfiguration();
        // 0 will be the leading motor; 1 will be the follower
        TalonFXConfiguration[] fireConfig = new TalonFXConfiguration[] {
            new TalonFXConfiguration(),
            new TalonFXConfiguration()
        };

        // Set current limits for all motors
        rotConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        for (TalonFXConfiguration config : fireConfig) {
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        rotConfig.Voltage.PeakForwardVoltage = TurretConstants.maxVoltage;
        rotConfig.Voltage.PeakReverseVoltage = -TurretConstants.maxVoltage;
    
        for (TalonFXConfiguration config : fireConfig) {
            config.Voltage.PeakForwardVoltage = TurretConstants.maxVoltage;
            config.Voltage.PeakReverseVoltage = -TurretConstants.maxVoltage;
        }

        // set state of the rotation motor
        rotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // set PID for rotation motor
        configureRotPID(
            rotMotor,
            TurretConstants.kRotPID[0], 
            TurretConstants.kRotPID[1], 
            TurretConstants.kRotPID[2]
        );

        configureFirePID(
            shootMotors,
            TurretConstants.kFirePID[0], 
            TurretConstants.kFirePID[1], 
            TurretConstants.kFirePID[2]
        );

        // Set the motors to coast on during idle
        rotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        for (TalonFXConfiguration config : fireConfig) {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        // Apply rotation motor configs
        rotMotor.getConfigurator().apply(rotConfig);

        // Apply firing motor configs
        for (int i = 0; i < shootMotors.length; i++) {
            shootMotors[i].getConfigurator().apply(fireConfig[i]);
        }

        // Set status update frequency to 10ms for shoot motors motors
        shootMotors[0].getMotorVoltage().setUpdateFrequency(100);
        shootMotors[1].getMotorVoltage().setUpdateFrequency(100);

        // Set the second firing motor to follow the first with opposite direction
        shootMotors[1].setControl(new Follower
            (shootMotors[0].getDeviceID(), MotorAlignmentValue.Opposed)
        );

        System.out.println("Shooter master ID: " + shootMotors[0].getDeviceID());
        System.out.println("Shooter follower ID: " + shootMotors[1].getDeviceID());

        System.out.println("TurretBeta motor configs applied!");
    }    
}
