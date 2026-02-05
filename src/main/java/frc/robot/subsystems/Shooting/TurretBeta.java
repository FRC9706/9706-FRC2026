package frc.robot.subsystems.Shooting;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.Tuning.LiveTuner;

public class TurretBeta extends SubsystemBase {
    // Singleton instance
    private static TurretBeta mInstance = null;
    public static TurretBeta getInstance(SwerveSubsystem drivebase) {
        if (mInstance == null) {
            mInstance = new TurretBeta(drivebase);
        }
        return mInstance;
    }

    // Hardware
    private SparkMax rotMotor;
    private RelativeEncoder rotEN;
    private SparkMaxConfig rotConfig;
    private EncoderConfig rotENConfig;

    // Drivetrain reference for field-relative control
    private SwerveSubsystem drivebase;

    // Motion profiling
    private TrapezoidProfile profile;
    private State setpoint = new State(0, 0);

    // Field tracking goals
    private Rotation2d goalFieldAng = Rotation2d.kZero;
    private double goalFieldVel = 0.0;
    private double lastGoalAng = 0.0;

    // Current turret position (rotations from encoder)
    private double turretAngRot;

    // Live tuning
    private final LiveTuner.TunableNumber kP;
    private final LiveTuner.TunableNumber kD;

    public static enum state {
        Idle,
        FieldTracking
    }

    private state currentState = state.Idle;

    public state getState() {
        return currentState;
    }

    public void setState(state newState) {
        currentState = newState;
    }

    public void configureMotors() {
        rotConfig = new SparkMaxConfig();
        rotENConfig = new EncoderConfig();

        rotConfig.inverted(false);
        rotConfig.idleMode(IdleMode.kCoast);
        rotConfig.smartCurrentLimit(10);
        
        // Encoder conversion factor (gear ratio)
        rotENConfig.positionConversionFactor(TurretConstants.gearRatio);

        rotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .outputRange(-1, 1);

        rotMotor.configure(rotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        System.out.println("TurretBeta motor configs applied!");
    }

    private TurretBeta(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;

        // Create motor and encoder
        rotMotor = new SparkMax(TurretConstants.rotationMotor, MotorType.kBrushless);
        rotEN = rotMotor.getEncoder();

        // Apply configs
        configureMotors();

        // Initialize motion profile
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                TurretConstants.maxVelRadPerSec,
                TurretConstants.maxAccelRadPerSec
            )
        );

        // Setup live tuning
        kP = LiveTuner.number("Turret/kP", TurretConstants.kRotPID[0]);
        kD = LiveTuner.number("Turret/kD", TurretConstants.kRotPID[2]);

        System.out.println("TurretBeta initialized!");
    }

    public void setFieldTarget(Rotation2d fieldAng, double fieldVel) {
        this.goalFieldAng = fieldAng;
        this.goalFieldVel = fieldVel;
        
        // Auto-switch to tracking when target is set
        if (currentState == state.Idle) {
            currentState = state.FieldTracking;
        }
    }

    private double findBestAng(double targetAngRad) {
        double bestAng = targetAngRad;
        double minLim = TurretConstants.turretRotLim * -1.0;
        double maxLim = TurretConstants.turretRotLim;

        // Try wrapping +/- 2 pi to find shortest path
        for (int i = -1; i <= 1; i++) {
            double candidate = targetAngRad + (Math.PI * 2.0 * i);
            if (candidate >= minLim && candidate <= maxLim) {
                if (Math.abs(candidate - lastGoalAng) < Math.abs(bestAng - lastGoalAng)) {
                    bestAng = candidate;
                }
            }
        }

        lastGoalAng = MathUtil.clamp(bestAng, minLim, maxLim);
        return lastGoalAng;
    }

    public void stopRotMotor() {
        rotMotor.stopMotor();
    }

    public void rotate(double power) {
        // Clamp power output
        power = MathUtil.clamp(power, -1.0, 1.0);

        // Safety limits
        double turretAngRad = Math.toRadians(turretAngRot * 360.0); // Convert rotations to radians
        if ((turretAngRad >= TurretConstants.turretRotLim && power > 0) 
            || (turretAngRad <= -TurretConstants.turretRotLim && power < 0)) {
            stopRotMotor();
            return;
        }

        rotMotor.set(power);
    }

    @Override
    public void periodic() {
        // Update encoder position (in rotations)
        turretAngRot = rotEN.getPosition();

        // State machine
        switch (currentState) {
            case Idle:
                stopRotMotor();
                // Hold current position in setpoint
                setpoint = new State(Math.toRadians(turretAngRot * 360.0), 0.0);
                break;

            case FieldTracking:
                // Get robot state from drivetrain
                Rotation2d robotHeading = drivebase.getPose().getRotation();
                double robotAngVel = drivebase.getFieldVelocity().omegaRadiansPerSecond;

                // Convert field goal to robot-relative goal
                Rotation2d robotRelGoal = goalFieldAng.minus(robotHeading);
                double robotRelVel = goalFieldVel - robotAngVel;

                // Find best angle (wrap-around logic)
                double bestAng = findBestAng(robotRelGoal.getRadians());

                // Calculate trapezoidal profile setpoint
                State goalState = new State(bestAng, robotRelVel);
                setpoint = profile.calculate(0.02, setpoint, goalState);

                // PD control
                double currentPosRad = Math.toRadians(turretAngRot * 360.0);
                double error = setpoint.position - currentPosRad;
                double output = (error * kP.get()) + (setpoint.velocity * kD.get());

                rotate(output);
                break;
        }
    }
}