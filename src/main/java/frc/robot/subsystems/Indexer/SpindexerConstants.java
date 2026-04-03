package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SpindexerConstants {
    // Motor IDs
    public static int motorID = 18;

    public static final TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                // Stator is disabled
                .withStatorCurrentLimitEnable(false)
        )
        .withVoltage(
            new VoltageConfigs()
                .withPeakForwardVoltage(10)
                .withPeakReverseVoltage(-10)
        )
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)   
        );

}
