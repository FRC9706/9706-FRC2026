// package frc.robot.subsystems.Shooting;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.ResetMode;
// import com.revrobotics.PersistMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.EncoderConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Vision.LimelightHelpers;
// import frc.robot.util.Tuning.LiveTuner;

// public class TurretNeo extends SubsystemBase {
//     // Create an instance for the TurretNeo
//     public static TurretNeo mInstance = null;
//     public static TurretNeo getInstance() {
//         if (mInstance == null) {
//             mInstance = new TurretNeo();
//         }
//         return mInstance;
//     }

//     // Initialize the motors for the turret
//     private SparkMax rotationMotor;
//         // Intialize stuff for the rotation motor
//         private RelativeEncoder rotEN;
//         private SparkMaxConfig rotConfig;
//         private EncoderConfig rotENConfig;
//         private final LiveTuner.TunableNumber trackingP;
    
//     // Initalize external CAN coder
    

//     // Initalize limelight variables
//     private double tx;
//     private boolean tv;

//     // Initalize turret variables
//     private double turretAngleDeg;

//     public static enum state {
//         Idle,
//         roamPos,
//         roamNeg,
//         Tracking,
//     }

//     private state currentState = state.Idle;

//     public state getState() {
//         return currentState;
//     }

//     public void setState(state newState) {
//         currentState = newState;
//     }

//     public void configureMotors() {
//         // Rotional motor configs
//         rotConfig = new SparkMaxConfig();
//             // Encoder config for the rotational motor
//             rotENConfig = new EncoderConfig();

//         // Configuration for slot 0 configs for the rotational motor
//         var rotSlot0Configs = rotConfig;
//         rotSlot0Configs.inverted(false);
//         rotSlot0Configs.idleMode(IdleMode.kCoast);
//         rotSlot0Configs.smartCurrentLimit(10);
//             // Configuration for the internal encoder of the rotational motor
//             rotENConfig
//             .positionConversionFactor(TurretConstants.gearRatio);

//         rotSlot0Configs.closedLoop 
//         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//         // .p(TurretConstants.kRotPID[0])
//         // .i(TurretConstants.kRotPID[1])
//         // .d(TurretConstants.kRotPID[2])
//         .outputRange(-1, 1);

//         // Apply the configurations to the motor
//         rotationMotor.configure(rotSlot0Configs, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

//         System.out.println("Motor configurations applied!");
//     }

//     public TurretNeo() {
//         // Create the motor objects
//         rotationMotor = new SparkMax(TurretConstants.rotationMotor, MotorType.kBrushless);
//             // Create the objects also needed for the rotation motor
//             rotEN = rotationMotor.getEncoder();;
        

//         // Apply the configurations for the motors
//         configureMotors();

//         // Setup live PID tuner for the motor's pid constants
//         LiveTuner.pid(
//         "Turret/Rotation",
//         TurretConstants.kRotPID[0], TurretConstants.kRotPID[1], TurretConstants.kRotPID[2],
//             (p, i, d) -> {
//                 SparkMaxConfig tunedConfig = new SparkMaxConfig();
//                 tunedConfig.apply(rotConfig);

//                 tunedConfig.closedLoop
//                     .p(p)
//                     .i(i)
//                     .d(d);

//                 rotationMotor.configure(
//                     tunedConfig,
//                     ResetMode.kNoResetSafeParameters,
//                     PersistMode.kNoPersistParameters
//                 );
//             }
//         );

//         // Setup live tuner for the tracking PID loop
//         trackingP = LiveTuner.number("Turret/TrackingP", TurretConstants.kTrackingP);

//         System.out.println("trackingP registered: " + trackingP.get());
//     }


//     public void stopRotationMotor() {
//         rotationMotor.stopMotor();
//     }

//     public void startAutoTrack() {
//         currentState = state.roamPos;
//     }

//     public void rotate(double power) {
//         power = Math.copySign((Math.min(Math.abs(power), TurretConstants.maxRotPower)), power);

//         if ((turretAngleDeg >= TurretConstants.turretRotLim) && power > 0 
//                 || (turretAngleDeg <= -TurretConstants.turretRotLim && power < 0)) {
//                     stopRotationMotor();
//                     return;
//         }
    
//         rotationMotor.set(power);
//     }

//     public void targetTracking() {
//         // No targets -> back to TagFinding
//         if (!tv) {
//             if (turretAngleDeg >= 0) {
//                 currentState = state.roamNeg;
//             } else if (turretAngleDeg < 0) {
//                 currentState = state.roamPos;
//             }
//         }

//         // Deadband to prevent motor overshoot
//         if (Math.abs(tx) < TurretConstants.Limelight.Tags.txDeadbandDeg) {
//             stopRotationMotor();
//             return;
//         }

//         // Simple proportional control
//         double rotationPower = tx * trackingP.get();

//         rotate(rotationPower);
//     }

//     public void getDistance() {
//         // For future distance calculations
//     }

//     @Override
//     public void periodic() {
//         // Asign limelight variables periodically to update continously
//         tx = LimelightHelpers.getTX("limelight-turret");
//         tv = LimelightHelpers.getTV("limelight-turret");

//         // Asign turret rotational values for calculations
//         turretAngleDeg = rotEN.getPosition();

//         // STATE LOGIC
//         switch (currentState) {
//             case Idle:
//                 stopRotationMotor();
//                 break;

//             case roamPos:
//                 if (tv) {
//                     currentState = state.Tracking;
//                 } else if (turretAngleDeg >= TurretConstants.turretRotLim) {
//                     currentState = state.roamNeg;
//                 } else if (turretAngleDeg < TurretConstants.turretRotLim) {
//                     rotate(TurretConstants.roamSpeed);
//                 }
//                 break;

//             case roamNeg:
//                 if (tv) {
//                     currentState = state.Tracking;
//                 } else if (turretAngleDeg <= -TurretConstants.turretRotLim) {
//                     currentState = state.roamPos;
//                 } else if (turretAngleDeg > -TurretConstants.turretRotLim) {
//                     rotate(-TurretConstants.roamSpeed);
//                 }
//                 break;

//             case Tracking:
//                 targetTracking();
//                 break;
//         }
//     }
// }
