package frc.robot.subsystems.Score;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Score.TurretModules.MotorConfigs;
import frc.robot.subsystems.Score.TurretModules.TurretMath;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.Tuning.LiveTuner;

public class TurretBeta extends SubsystemBase {
    // Singleton instance
    private static TurretBeta mInstance = null;
    public static TurretBeta getInstance(SwerveSubsystem drivebase, TurretMath turretMath) {
        if (mInstance == null) {
            mInstance = new TurretBeta(drivebase, turretMath);
        }
        return mInstance;
    }

    // Hardware
    private TalonFX rotMotor;
    private CANcoder rotEN;

    private TalonFX[] shootMotors;

    // Drivetrain reference for field-relative control
    private SwerveSubsystem drivebase;

    // Get turretMath reference for calculating different trajectorial points
    private TurretMath turretMath;

    // Current turret position (rotations from encoder)
    private double turretPos = 404.0;

    // Motion profiling
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // Temp logging
    LoggedNetworkNumber loggedTurretPos = 
    new LoggedNetworkNumber("Turret/loggedTurretAng", 0.0);

    LoggedNetworkNumber loggedKrakenRot = 
    new LoggedNetworkNumber("Turret/loggedKrakenRot", 0.0);

    LoggedNetworkNumber loggedRotEN = 
    new LoggedNetworkNumber("Turret/loggedRotEN", 0.0);

    LoggedNetworkBoolean ARTStatus = 
    new LoggedNetworkBoolean("Turret/ARTStatus", false);

    public void configureRotPID (
        double p, double i, double d,
        double s, double v, double a, 
        double cruiseVel, double accel, double jerk
    ) {
        MotorConfigs.configureRotPID(
            rotMotor, p, i, d, 
            s, v, a, 
            cruiseVel, accel, jerk);
    }

    public void configureFirePID(double p, double i, double d) {
        MotorConfigs.configureFirePID(shootMotors, p, i, d);
    }

    private TurretBeta(SwerveSubsystem drivebase, TurretMath turretMath) {
        this.drivebase = drivebase;
        this.turretMath = turretMath;
        
        // Create motor and encoder
        rotMotor = new TalonFX(TurretConstants.rotationMotor);
            rotEN = new CANcoder(TurretConstants.rotationCanCoder);
        shootMotors = new TalonFX[] {
            new TalonFX(TurretConstants.firingMotors[0]),
            new TalonFX(TurretConstants.firingMotors[1])
        };

        // Apply configs
        MotorConfigs.configureMotors(rotMotor, shootMotors);

        // Setup live tuning for PID
        LiveTuner.pidMagicMotion("Turret/RotationPID", 
            TurretConstants.kRotPID[0], 
            TurretConstants.kRotPID[1], 
            TurretConstants.kRotPID[2], 
            TurretConstants.kRotPID[3],
            TurretConstants.kRotPID[4],
            TurretConstants.kRotPID[5],
            TurretConstants.kRotPID[6],
            TurretConstants.kRotPID[7],
            TurretConstants.kRotPID[8],
            this::configureRotPID
        );

        LiveTuner.pid("Turret/FirePID", 
            TurretConstants.kFirePID[0], 
            TurretConstants.kFirePID[1], 
            TurretConstants.kFirePID[2], 
            this::configureFirePID
        );

        System.out.println("TurretBeta initialized!");
    }

    // --------------------------------------------------------------
    // Get functions
    // --------------------------------------------------------------
    public double getKrakenRot() {
        return (rotMotor.getPosition().getValueAsDouble());
    }

    public double getTurretPos() {
        return turretPos;
    }

    // --------------------------------------------------------------
    // Motor utilities
    // --------------------------------------------------------------

    public void stopRotMotor() {
        rotMotor.stopMotor();
    }

    public void stopShootMotors() {
        shootMotors[0].stopMotor();
    }

    // --------------------------------------------------------------
    // Position utilities
    // --------------------------------------------------------------
    
    public void resetRotEncoderPositons() {
        rotMotor.setPosition(0);
        rotEN.setPosition(0);

        System.out.println("TurretBeta encoders reset!");
        System.out.println("Motor Pos: " + rotMotor.getPosition().getValueAsDouble());
        System.out.println("Encoder Pos: " + rotEN.getAbsolutePosition().getValueAsDouble());
    }

    public void updateTurretPos() {
        double tempTurretPos =
            turretMath.calculateAndrewPos(
            getKrakenRot(), 
            rotEN.getPosition().getValueAsDouble());

        if (tempTurretPos >= 0.5) {
            tempTurretPos -= 1.0;
        }

        tempTurretPos += Math.floor((getKrakenRot() / TurretConstants.rotMotorGearRatio) + 0.5);

        turretPos = tempTurretPos;
    }

    // --------------------------------------------------------------
    // Movement utilities
    // --------------------------------------------------------------

    public void shoot(double rpm) {
        // rpm -> velocity
        double vel = rpm/60;
        System.out.println("Shooting at RPM: " + rpm + " Velocity: " + vel);

        // velocity contorl
        final VelocityVoltage m_request = new VelocityVoltage(vel).withSlot(0);
        shootMotors[0].setControl(m_request);
    
        // Log BOTH motors + follower status
        System.out.println("Master: " + shootMotors[0].getVelocity().getValueAsDouble() + 
                      "Follower: " + shootMotors[1].getVelocity().getValueAsDouble());
    }

    public void moveTurretToPos(double desiredPos) {
        double neededMotorRotations = (desiredPos - turretPos) * TurretConstants.rotMotorGearRatio;
        System.out.println("Moving turret to " + desiredPos + ". Need Kraken rotations: " + neededMotorRotations + " (Kraken target: " + (getKrakenRot() + neededMotorRotations) + " rotations)");
        double targetMotorPos = getKrakenRot() + neededMotorRotations;

        
        rotMotor.setControl(m_request.withPosition(targetMotorPos));
    }

    public void smartMoveTurretToPos(double desiredPos) {
        moveTurretToPos(turretMath.findFastestPos(
            desiredPos,
            turretPos,
            TurretConstants.turretRotLim
        ));
    }

    @Override
    public void periodic() {
        // Update encoder position (in ROTATIONS from Andrew CRT)
        updateTurretPos();

        // Temp logging
        loggedTurretPos.set(turretPos);
        loggedKrakenRot.set(getKrakenRot());
        loggedRotEN.set(rotEN.getPosition().getValueAsDouble());

        // Add safety againt invalid return on ART (Andrew Remainder Theorem)
        if (turretPos <= 400) {
            ARTStatus.set(true);
            return;
        } if (turretPos >= 400) {
            System.out.println("Turret: Andrew remainder theorm FAILED!");
            stopRotMotor();
            ARTStatus.set(false);
        }
    }
}