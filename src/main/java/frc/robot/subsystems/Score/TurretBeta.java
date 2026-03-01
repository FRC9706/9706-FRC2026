package frc.robot.subsystems.Score;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.Tuning.LiveTuner;
import frc.robot.subsystems.Score.TurretModules.Trajectory;
import frc.robot.subsystems.Score.TurretModules.MotorConfigs;

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

    private TalonFX[] shootMotors;

    // Drivetrain reference for field-relative control
    private SwerveSubsystem drivebase;
    // Get trajectory reference for calculating different trajectorial points
    private Trajectory trajectory;

    // Motion profiling
    private TrapezoidProfile profile;
    private State setpoint = new State(0, 0);

    // Field tracking goals
    private Rotation2d goalFieldAng = Rotation2d.kZero;
    private double goalFieldVelRotPerSec = 0.0;
    private double lastGoalAng = 0.0;

    // Current turret position (rotations from encoder)
    private double turretAngRot;

    // Live tuning
    private final LiveTuner.TunableNumber kP;
    private final LiveTuner.TunableNumber kV;

    // Temp logging
    LoggedNetworkNumber loggedTurretAng = 
    new LoggedNetworkNumber("Turret/loggedTurretAng", 0.0);

    LoggedNetworkNumber loggedKrakenRot = 
    new LoggedNetworkNumber("Turret/loggedKrakenRot", 0.0);

    LoggedNetworkNumber loggedRotEN = 
    new LoggedNetworkNumber("Turret/loggedRotEN", 0.0);

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
        MotorConfigs.configureRotPID(rotMotor, p, i, d);
    }

    public void configureFirePID(double p, double i, double d) {
        MotorConfigs.configureFirePID(shootMotors, p, i, d);
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
        MotorConfigs.configureMotors(rotMotor, shootMotors);

        // Initialize motion profile
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                TurretConstants.maxVelRotPerSec,
                TurretConstants.maxAccelRotPerSec
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

    public void stopRotMotor() {
        rotMotor.stopMotor();
    }

    public void stopShootMotors() {
        shootMotors[0].stopMotor();
    }

    public void shootRPM(double rpm) {
        double vel = rpm/60;
        shoot(vel);
    }

    public void shoot(double vel) {
        final VelocityVoltage m_request = new VelocityVoltage(vel).withSlot(0);
        shootMotors[0].setControl(m_request);
    
        // Log BOTH motors + follower status
        System.out.println("Master: " + shootMotors[0].getVelocity().getValueAsDouble() + 
                      "Follower: " + shootMotors[1].getVelocity().getValueAsDouble());
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

    public void setFieldTarget(Rotation2d fieldAng, double fieldVelRotPerSec) {
        this.goalFieldAng = fieldAng;
        this.goalFieldVelRotPerSec = fieldVelRotPerSec;
        
        // Auto-switch to tracking when target is set
        if (currentState == state.Idle) {
            currentState = state.FieldTracking;
        }
    }

    private double findBestAngRot(double targetAngRot) {
        double bestAng = targetAngRot;
        double minLim = -TurretConstants.turretRotLim;
        double maxLim = TurretConstants.turretRotLim;

        // Try wrapping ±1.0 rotation
        for (int i = -1; i <= 1; i++) {
            double candidate = targetAngRot + (1.0 * i);  // ±1 full rotation
            if (candidate >= minLim && candidate <= maxLim) {
                if (Math.abs(candidate - lastGoalAng) < Math.abs(bestAng - lastGoalAng)) {
                    bestAng = candidate;
                }
            }
        }

        lastGoalAng = MathUtil.clamp(bestAng, minLim, maxLim);
        return lastGoalAng;
    }

    @Override
    public void periodic() {
        // Update encoder position (in ROTATIONS from Andrew CRT)
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
                // Hold current position in setpoint (ROTATIONS)
                setpoint = new State(turretAngRot, 0.0);
                break;

            case FieldTracking:
                // Get robot state from drivetrain
                Rotation2d robotHeading = drivebase.getPose().getRotation();
                double robotAngVelRad = drivebase.getFieldVelocity().omegaRadiansPerSecond;

                // Convert robot heading and velocity to ROTATIONS
                double robotHeadingRot = robotHeading.getRotations();
                double robotAngVelRot = robotAngVelRad / (2.0 * Math.PI);  // rad/s → rot/s

                // Get goal in ROTATIONS
                double goalFieldRot = goalFieldAng.getRotations();

                // Convert field goal to robot-relative goal (ROTATIONS)
                double robotRelGoalRot = goalFieldRot - robotHeadingRot;
                double robotRelVelRot = goalFieldVelRotPerSec - robotAngVelRot;

                // Find best angle (wrap-around logic in ROTATIONS)
                double bestAngRot = findBestAngRot(robotRelGoalRot);

                // Calculate trapezoidal profile setpoint (ROTATIONS)
                State goalState = new State(bestAngRot, robotRelVelRot);
                setpoint = profile.calculate(0.02, setpoint, goalState);

                // PV control (ROTATIONS)
                double posError = setpoint.position - turretAngRot;
                double output = (posError * kP.get()) + (setpoint.velocity * kV.get());

                rotate(output);
                break;
        }
    }
}