// package frc.robot.subsystems.Score;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Score.TurretModules.MotorConfigs;
// import frc.robot.subsystems.Score.TurretModules.TurretMath;
// import frc.robot.subsystems.Swerve.SwerveSubsystem;
// import lib.Geometry.Translation2d;
// import lib.Networking.DynamicInputs;

// public class TurretBeta extends SubsystemBase {
//     // Singleton instance
//     private static TurretBeta mInstance = null;
//     public static synchronized TurretBeta getInstance(SwerveSubsystem drivebase, TurretMath turretMath) {
//         if (mInstance == null) {
//             mInstance = new TurretBeta(drivebase, turretMath);
//         }
//         return mInstance;
//     }

//     // initalize hardware variables
//     private TalonFX rotMotor;
//     private CANcoder rotEN;

//     private TalonFX[] shootMotors;

//     // Drivetrain reference for field-relative control
//     private final SwerveSubsystem mDrivebase;

//     // Get turretMath reference for calculating different trajectorial points
//     private final TurretMath mTurretMath;

//     // Current turret position (rotations from encoder)
//     private double turretPos = 404.0;
//     // private boolean isTurretInitialized = false;

//     // Motion profiling
//     private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

//     public void configureRotPID (
//         double p, double i, double d,
//         double s, double v, double a, 
//         double cruiseVel, double accel, double jerk) {
//         MotorConfigs.configureRotPID(
//             rotMotor, p, i, d, 
//             s, v, a, 
//             cruiseVel, accel, jerk);
//     }

//     public void configureFirePID(double p, double i, double d) {
//         MotorConfigs.configureFirePID(shootMotors, p, i, d);
//     }

//     public TurretBeta(SwerveSubsystem drivebaseInst, TurretMath turretMathInst) {
//         mDrivebase = drivebaseInst;
//         mTurretMath = turretMathInst;
        
//         // Create motor and encoder
//         rotMotor = new TalonFX(TurretConstants.rotationMotor);
//             rotEN = new CANcoder(TurretConstants.rotationCanCoder);
//         shootMotors = new TalonFX[] {
//             new TalonFX(TurretConstants.firingMotors[0]),
//             new TalonFX(TurretConstants.firingMotors[1])
//         };

//         // Apply configs
//         MotorConfigs.configureMotors(rotMotor, shootMotors);

//         // Setup live tuning for PID
//         DynamicInputs.pidMagicMotion("Turret/RotationPID", 
//             TurretConstants.kRotPID[0], 
//             TurretConstants.kRotPID[1], 
//             TurretConstants.kRotPID[2], 
//             TurretConstants.kRotPID[3],
//             TurretConstants.kRotPID[4],
//             TurretConstants.kRotPID[5],
//             TurretConstants.kRotPID[6],
//             TurretConstants.kRotPID[7],
//             TurretConstants.kRotPID[8],
//             this::configureRotPID
//         );

//         DynamicInputs.pid("Turret/FirePID", 
//             TurretConstants.kFirePID[0], 
//             TurretConstants.kFirePID[1], 
//             TurretConstants.kFirePID[2], 
//             this::configureFirePID
//         );

//         System.out.println("TurretBeta initialized!");
//     }

//     // --------------------------------------------------------------
//     // Get functions
//     // --------------------------------------------------------------
//     public double getKrakenRot() {
//         return (rotMotor.getPosition().getValueAsDouble());
//     }

//     public double getTurretPos() {
//         return turretPos;
//     }

//     // --------------------------------------------------------------
//     // Motor utilities
//     // --------------------------------------------------------------

//     public void stopRotMotor() {
//         rotMotor.stopMotor();
//     }

//     public void stopShootMotors() {
//         shootMotors[0].stopMotor();
//     }

//     // --------------------------------------------------------------
//     // Position utilities
//     // --------------------------------------------------------------
//     @SuppressWarnings("unused")
//     public void resetRotEncoderPositons() {
//         if ((TurretConstants.extEncoderTeeth % 2) == 0) {
//         rotMotor.setPosition(0);
//         } else {
//         rotMotor.setPosition(0.5);
//         }

//         if ((TurretConstants.rotMotorTeeth % 2) == 0) {
//         rotEN.setPosition(0);
//         } else {
//         rotEN.setPosition(0.5);
//         }

//         System.out.println("TurretBeta encoders reset!");
//         System.out.println("Motor Pos: " + rotMotor.getPosition().getValueAsDouble());
//         System.out.println("Encoder Pos: " + rotEN.getAbsolutePosition().getValueAsDouble());
//         updateTurretPos();
//         initTurret(true);
//     }

//     public void updateTurretPos() {
//         double tempTurretPos =
//             mTurretMath.newNewBRT(
//             getKrakenRot(), 
//             rotEN.getPosition().getValueAsDouble());

//         // tempTurretPos -= TurretConstants.turretRotLim;

//         Logger.recordOutput("Turret/Math/ART/turretAngRot", tempTurretPos);

//         turretPos = tempTurretPos;
//     }

//     public void initTurret(boolean override) {
//         // if (!isTurretInitialized || override){
//             rotMotor.setPosition(0);
//             rotEN.setPosition(0);
//             //     isTurretInitialized = true;
//             // }
//         //     if (!(isTurretInitialized) || override) {
//         //     if (turretPos < 400) {
//         //         rotMotor.setPosition((turretPos + 0.5)*TurretConstants.rotMotorGearRatio);
//         //         rotEN.setPosition((turretPos + 0.5)*TurretConstants.extEncoderGearRatio);
//         //         isTurretInitialized = true;
//         //         System.out.println("Turret has been initialized!");
//         //     }
//         // }
//     }

//     // --------------------------------------------------------------
//     // Movement utilities
//     // --------------------------------------------------------------

//     public void shoot(double rpm) {
//         // rpm -> velocity
//         double vel = rpm/60;
//         System.out.println("Shooting at RPM: " + rpm + " Velocity: " + vel);

//         // velocity contorl
//         final VelocityVoltage m_request = new VelocityVoltage(vel).withSlot(0);
//         shootMotors[0].setControl(m_request);
    
//         // Log BOTH motors + follower status
//         System.out.println("Master: " + shootMotors[0].getVelocity().getValueAsDouble() + 
//                       "Follower: " + shootMotors[1].getVelocity().getValueAsDouble());
//     }

//     public void rotate(double power) {
//         // Clamp power output
//         power = MathUtil.clamp(power, -1.0, 1.0);
//         rotMotor.set(power);
//     }

//     public void moveTurretToPos(double desiredPos) {
//         double neededTurretRotations = desiredPos - turretPos;
//         double neededMotorRotations = neededTurretRotations * TurretConstants.rotMotorGearRatio;

//         double targetMotorPos = getKrakenRot() + neededMotorRotations;

//         Logger.recordOutput("Turret/Movement/desiredPos", desiredPos);
//         Logger.recordOutput("Turret/Movement/turretPos", turretPos);
//         Logger.recordOutput("Turret/Movement/neededTurretRot", neededTurretRotations);
//         Logger.recordOutput("Turret/Movement/neededMotorRot", neededMotorRotations);
//         Logger.recordOutput("Turret/Movement/krakenTarget", targetMotorPos);

//         Logger.recordOutput(
//             "Turret/Movement/targetOutOfBounds",
//             targetMotorPos < 0 || targetMotorPos > (TurretConstants.rotOfFreedom * TurretConstants.rotMotorGearRatio)
//         );

//         if (targetMotorPos < 0 || targetMotorPos > (TurretConstants.rotOfFreedom * TurretConstants.rotMotorGearRatio)) {
//             System.out.println("Turret trying to move out of bounds!");
//             stopRotMotor();
//             return;
//         }

//         rotMotor.setControl(m_request.withPosition(targetMotorPos));
//     }

//     public void smartMoveTurretToPos(double desiredPos) {
//         moveTurretToPos(mTurretMath.findFastestPos(
//             desiredPos,
//             turretPos,
//             TurretConstants.safeTurretRotLim
//         ));
//     }

//     public void smartMoveToTranslation(Translation2d target) {
//         smartMoveTurretToPos(mTurretMath.getTurretOffsetToTranslation(target));
//     }

//     public void smartMoveToHub() {
//         smartMoveTurretToPos(mTurretMath.getTurretOffsetRot());
//     }

//     @Override
//     public void periodic() {
//         // intialized encoders for turret
//         // initTurret(false);
        
//         // Update encoder position (in ROTATIONS from Andrew CRT)
//         updateTurretPos();

//         // Temp logging
//         Logger.recordOutput("Turret/Math/ART/krakenRot", getKrakenRot());

//         Logger.recordOutput(
//             "Turret/Math/ART/extEncoderRot", 
//             rotEN.getPosition().getValueAsDouble()
//         );
        
//         Logger.recordOutput("Turret/hubDistance", mTurretMath.getRobotDistHub());

//         // Add safety againt invalid return on IDK_RT (??? Remainder Theorem)
//         if (turretPos <= 400) {
//             Logger.recordOutput("Turret/Math/ART/status", true);
//             return;
//         } if (turretPos >= 400) {
//             System.out.println("Turret: Andrew remainder theorm FAILED!");
//             stopRotMotor();
//             Logger.recordOutput("Turret/Math/ART/status", false);
//         }
//     }
// }