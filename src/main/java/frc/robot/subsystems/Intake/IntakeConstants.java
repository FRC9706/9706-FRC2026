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
    public static int rollerMotorID = 15;
    public static int[] extMotorIDs = {16, 17};

    // Motor variables
    public static int maxVoltage = 10;

    // Motor positions for extention motor
    public static final double extPosTolerance = 0.5;
    public static final double retractedPos = 0;
    public static final double extendedPos = 4.1895947219511;
    public static final double[] wiggleRange = {3,2};


    public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true)
                // Disable stator
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
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)   
        )
        .withFeedback(
            new FeedbackConfigs().withSensorToMechanismRatio(1)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(12)
                .withKD(0.25)
                .withKS(0.25)
                .withKV(0.10)
                .withKA(0.01)
        )
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(40)
                .withMotionMagicAcceleration(120)
                .withMotionMagicJerk(1600)
        );

    public static final TalonFXConfiguration extConfig2 = extConfig.clone()
        .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(10)
                .withKD(0.25)
                .withKS(0.25)
                .withKV(0.10)
                .withKA(0.01)
        );
    
}
