package frc.robot.subsystems.Score.Turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class TurretConstants {
    // Motor IDs
    public static final int[] firingMotors = {10, 11};
    public static final int rotationMotor = 9;
    public static final int pitchMotor = 3;

    // CAN coder IDs
    public static final int rotationCanCoder = 5;
    public static final int pitchCanCoder = 6;

    // Rotational motor variables
    public static final int maxVoltage = 10;

    // Rotional Gear variables
    public static final double turretGearsGCF = 6d;
    public static final double rotMotorTeeth = 42d;
    public static final double extEncoderTeeth = 36d;
    public static final double centerGearTeeth = 170d; // Its going to be okay

    public static final double rotMotorGearRatio = (double)centerGearTeeth/(double)rotMotorTeeth;
    public static final double extEncoderGearRatio = (double)centerGearTeeth/(double)extEncoderTeeth;
    

    // Rotational motor settings
    public static final double roamSpeed = 0.15;
    public static final double maxRotPower = 0.5;
    public static final double rotOfFreedom = (double)252/(double)170; // Encoders and Center Gear can NEVER be Allowed to Rotate Freely Outside of This Range
    public static final double turretRotLim = rotOfFreedom / 2;
    public static final double safeTurretRotLim = turretRotLim -0.1;
    
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
                    .withKP(150)
                )
            .withMotionMagic(
              new MotionMagicConfigs()
              .withMotionMagicCruiseVelocity(Units.degreesToRotations(360*4))
              .withMotionMagicAcceleration(Units.degreesToRotations(360*8))
            );
}
