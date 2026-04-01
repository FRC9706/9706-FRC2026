package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    // Motor ID
    public static int rollerMotorID = 14;
    public static int[] extMotorIDs = {15, 16};

    // Motor variables
    public static int maxVoltage = 10;

    // Motor positions for extention motor
    public static final double retractedPos = 0;
    public static final double extendedPos = 3;
    public static final double[] wiggleRange = {3,2};


    public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration()
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
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)   
        );

        public static final TalonFXConfiguration extConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(25)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(25)
                .withStatorCurrentLimitEnable(true)
        )
        .withVoltage(
            new VoltageConfigs()
                .withPeakForwardVoltage(10)
                .withPeakReverseVoltage(-10)
        )
        .withFeedback(
            new FeedbackConfigs().withSensorToMechanismRatio(Math.PI)
        )
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)   
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(10)
        )
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(6)
                .withMotionMagicAcceleration(6)
        );

    public static final TalonFXConfiguration extConfig2 = extConfig
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        );
    
}
