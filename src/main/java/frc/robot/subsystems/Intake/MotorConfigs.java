package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorConfigs {
     public static void configureMotor(TalonFX motor) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Set current limits for the motor
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        motorConfig.Voltage.PeakForwardVoltage = IntakeConstants.maxVoltage;
        motorConfig.Voltage.PeakReverseVoltage = -IntakeConstants.maxVoltage;

        // set state of the motor
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Set the motor to brake on during idle
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply motor configs
        motor.getConfigurator().apply(motorConfig);

        System.out.println("Intake motor configs applied!");
    }    
}
