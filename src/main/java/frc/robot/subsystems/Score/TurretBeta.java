package frc.robot.subsystems.Score;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    public static TurretBeta getInstance(SwerveSubsystem drivebase, Trajectory trajectory) {
        if (mInstance == null) {
            mInstance = new TurretBeta(drivebase, trajectory);
        }
        return mInstance;
    }

    // Hardware
    private TalonFX rotMotor;
    private CANcoder rotEN;
        private TalonFXConfiguration rotConfig;

    private TalonFX[] shootMotors;
        private TalonFXConfiguration[] fireConfig;

    // Drivetrain reference for field-relative control
    private SwerveSubsystem drivebase;
    // Get trajectory reference for calculating different trajectorial points
    private Trajectory trajectory;

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
    private final LiveTuner.TunableNumber kV;

    // Temp logging
    LoggedNetworkNumber loggedTurretAng = new LoggedNetworkNumber("Turret", 0.0);
    LoggedNetworkNumber loggedKrakenRot = new LoggedNetworkNumber("Turret", 0.0);
    LoggedNetworkNumber loggedRotEN = new LoggedNetworkNumber("Turret", 0.0);

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

    public void configureRotPID(double p, double i, double d) {
        // PID configurator for the rotational motor
        var rotSlot0Configs = rotConfig.Slot0;
        rotSlot0Configs.kP = p;
        rotSlot0Configs.kI = i;
        rotSlot0Configs.kD = d;

        rotMotor.getConfigurator().apply(rotSlot0Configs);
    }

    public void configureFirePID(double p, double i, double d) {
        // PID configurator for the firing motors (assumes both have same PID)
        for (TalonFXConfiguration config : fireConfig) {
            var slot0Configs = config.Slot0;
            slot0Configs.kP = p;
            slot0Configs.kI = i;
            slot0Configs.kD = d;
        }

        for (int n = 0; n < shootMotors.length; n++) {
            shootMotors[n].getConfigurator().apply(fireConfig[n]);
        }
    }

    public void configureMotors() {
        rotConfig = new TalonFXConfiguration();
        // 0 will be the leading motor; 1 will be the follower
        fireConfig = new TalonFXConfiguration[] {
            new TalonFXConfiguration(),
            new TalonFXConfiguration()
        };

        // set state of the rotation motor
        rotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // set PID for rotation motor
        configureRotPID(
            TurretConstants.kRotPID[0], 
            TurretConstants.kRotPID[1], 
            TurretConstants.kRotPID[2]
        );

        configureFirePID(
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

        // Set the second firing motor to follow the first with opposite direction
        shootMotors[1].setControl(new Follower
            (shootMotors[0].getDeviceID(), MotorAlignmentValue.Opposed)
        );

        System.out.println("Shooter master ID: " + shootMotors[0].getDeviceID());
        System.out.println("Shooter follower ID: " + shootMotors[1].getDeviceID());


        System.out.println("TurretBeta motor configs applied!");
    }

    private TurretBeta(SwerveSubsystem drivebase, Trajectory trajectory) {
        this.drivebase = drivebase;
        this.trajectory = trajectory;

        // Create motor and encoder
        rotMotor = new TalonFX(TurretConstants.rotationMotor);
            rotEN = new CANcoder(TurretConstants.rotationCanCoder);
        shootMotors = new TalonFX[] {
            new TalonFX(TurretConstants.firingMotors[0]),
            new TalonFX(TurretConstants.firingMotors[1])
        };

        // Apply configs
        configureMotors();

        // Initialize motion profile
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                TurretConstants.maxVelRadPerSec,
                TurretConstants.maxAccelRadPerSec
            )
        );

        // Setup live tuning for PID
        LiveTuner.pid("Turret/RotationPID", 
            TurretConstants.kRotPID[0], 
            TurretConstants.kRotPID[1], 
            TurretConstants.kRotPID[2], 
            this::configureRotPID
        );

        LiveTuner.pid("Turret/FirePID", 
            TurretConstants.kFirePID[0], 
            TurretConstants.kFirePID[1], 
            TurretConstants.kFirePID[2], 
            this::configureFirePID
        );

        // Setup live tuning (for rotaional tracking variables)
        kP = LiveTuner.number("Turret/TrackingPID/kP", TurretConstants.kRotTrackingP);
        kV = LiveTuner.number("Turret/TrackingPID/kV", TurretConstants.kRotTrackingV);

        System.out.println("TurretBeta initialized!");
    }

    public double getKrakenRot() {
        return (rotMotor.getPosition().getValueAsDouble() + 0.279785) % 1.0;
    }

    public double getTurretAngleRot() {
        return turretAngRot;
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

    public void stopShootMotors() {
        for (TalonFX motor : shootMotors) {
            motor.stopMotor();
        }
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

    public void shoot(double vel) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shootMotors[0].setControl(m_request.withVelocity(vel));
    }

    @Override
    public void periodic() {
        // Update encoder position (in rotations)
        turretAngRot = trajectory.calculateAndrewPos(
            getKrakenRot(), 
            rotEN.getPosition().getValueAsDouble()
        );

        // Temp logging
        loggedTurretAng.set(turretAngRot);
        loggedKrakenRot.set(getKrakenRot());
        loggedRotEN.set(rotEN.getPosition().getValueAsDouble());

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

                // PV control
                double currentPosRad = Math.toRadians(turretAngRot * 360.0);
                double error = setpoint.position - currentPosRad;
                double output = (error * kP.get()) + (setpoint.velocity * kV.get());

                rotate(output);
                break;
        }
    }
}