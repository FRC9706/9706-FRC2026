package frc.robot.subsystems.Score.Flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelConstants {

    // Motor ID
    public static final int[] motorIDs = {13,12};
    
    public static final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
              )
            .withCurrentLimits(
              new CurrentLimitsConfigs()
              .withSupplyCurrentLimit(40)
              .withSupplyCurrentLimitEnable(true)
              .withStatorCurrentLimit(40)
              .withStatorCurrentLimitEnable(true)
            )
            .withVoltage(
              new VoltageConfigs()
              .withPeakForwardVoltage(10)
              .withPeakReverseVoltage(-10)
            )
            .withFeedback(
              new FeedbackConfigs()
              .withSensorToMechanismRatio(
           170d/36d 
              )
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(10)
            );

}
