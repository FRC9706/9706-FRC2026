package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorConfigs {
    // Static utility class for configuring spindexer motors
    public static void configureSpinPID(TalonFX motor, double p, double i, double d) {
        // PID configurator for the spindexer
            var spinConfig = new TalonFXConfiguration();
            motor.getConfigurator().refresh(spinConfig);
            var slot0Configs = spinConfig.Slot0;
            slot0Configs.kP = p;
            slot0Configs.kI = i;
            slot0Configs.kD = d;
            slot0Configs.kV = 0.12;
            slot0Configs.kS = 0.25;

            motor.getConfigurator().apply(spinConfig);
        }

    public static void configureMotors(TalonFX motor) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Set current limits for the motor
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        motorConfig.Voltage.PeakForwardVoltage = SpindexerConstants.maxVoltage;
        motorConfig.Voltage.PeakReverseVoltage = -SpindexerConstants.maxVoltage;

        // set state of the motor
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Set the motor to brake on during idle
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply PID configs for the motor
        configureSpinPID(motor, 
            SpindexerConstants.kPID[0], // kP
            SpindexerConstants.kPID[1], // kI
            SpindexerConstants.kPID[2] // kD
        );

        // Apply motor configs
        motor.getConfigurator().apply(motorConfig);

        System.out.println("Spindexer motor configs applied!");
    }    
}
