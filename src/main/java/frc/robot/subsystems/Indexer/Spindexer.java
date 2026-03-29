// package frc.robot.subsystems.Indexer;

// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import lib.Networking.DynamicInputs;

// public class Spindexer extends SubsystemBase{
//     // Singleton instance
//     private static Spindexer mInstance = null;
//     public static synchronized Spindexer getInstance() {
//         if (mInstance == null) {
//             mInstance = new Spindexer();
//         }
//         return mInstance;
//     }

//     // Hardware
//     TalonFX motor;

//     public Spindexer() {
//         // Intialize Motors
//         motor = new TalonFX(SpindexerConstants.motorID);

//         // Configure motors
//         MotorConfigs.configureMotors(motor);

//         // Setup live tuning for spindexer PID
//         DynamicInputs.pid(
//             "Spindexer/PID", 
//             SpindexerConstants.kPID[0], 
//             SpindexerConstants.kPID[1], 
//             SpindexerConstants.kPID[2],
//             (p, i, d) -> MotorConfigs.configureSpinPID(motor, p, i, d)
//         );
//     }

//     public void stopDexer() {
//         motor.stopMotor();
//     }
    
//     public void spinDexer(double rpm) {
//         // rpm -> velocity
//         double vel = rpm/60;
//         System.out.println("Spinning at RPM: " + rpm + " Velocity: " + vel);

//         // velocity control
//         final VelocityVoltage m_request = new VelocityVoltage(vel).withSlot(0);
//         motor.setControl(m_request);
//     }

//     @Override
//     public void periodic() {
//         // Nothing here yet
//     }
// }
