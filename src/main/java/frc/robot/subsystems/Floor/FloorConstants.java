package frc.robot.subsystems.Floor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FloorConstants {

    public static final int floorMotorID = 19;

    public static final TalonFXConfiguration floorConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
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
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        );
}
