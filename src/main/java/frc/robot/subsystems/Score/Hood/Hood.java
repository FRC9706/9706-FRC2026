// package frc.robot.subsystems.Score.Hood;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Hopper.HopperConstants;
// import lib.Networking.DynamicInputs;

// public class Hood extends SubsystemBase {
//     // Singleton instance
//     private static Hood mInstance = null;
//     public static synchronized Hood getInstance() {
//         if (mInstance == null) {
//             mInstance = new Hood();
//         }
//         return mInstance;
//     }

//     // Motor variables
//     private TalonFX hoodMotor;
//     private MotionMagicVoltage m_request;

//     public Hood() {
//         // initalize motors
//         hoodMotor = new TalonFX(HoodConstants.hoodMotorID);

//         // Configure the hood motor
//         MotorConfigs.configurHoodMotor(hoodMotor);

//         // setup live tuner for the hood motor
//         DynamicInputs.pidMagicMotion("Turret/HoodPID", 
//             HopperConstants.kPID[0], 
//             HopperConstants.kPID[1], 
//             HopperConstants.kPID[2], 
//             HopperConstants.kPID[3],
//             HopperConstants.kPID[4],
//             HopperConstants.kPID[5],
//             HopperConstants.kPID[6],
//             HopperConstants.kPID[7],
//             HopperConstants.kPID[8],
//             (p, i, d, 
//                 s, y, b, a, u, 
//             ok) -> 
//                 MotorConfigs.configureHoodPID(
//                     hoodMotor, p, i, d, 
//                         s, y, b, a, u, 
//                     ok)
//         );
//     }
//     public double getHoodPos() {
//         return hoodMotor.getPosition().getValueAsDouble();
//     }

//     public void stopHoodMotor() {
//         hoodMotor.stopMotor();
//     }

//     public void moveHopperToPos(double pos) {
//         m_request = new MotionMagicVoltage(0).withPosition(pos);
//         hoodMotor.setControl(m_request);
//     }

//     public boolean hasReachedPos(double pos) {
//         boolean hasReachedPos = (Math.abs(getHoodPos() - pos) <= HoodConstants.posTolerance);
//         return hasReachedPos;
//     }

//     @Override
//     public void periodic() {
//         Logger.recordOutput("Turret/Hood/HoodPos", getHoodPos());
//     }
// }
