package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Singleton instance
    private static Intake mInstance = null;
    public static synchronized Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    // init motor variables
    private TalonFX motor;

    public Intake() {
        // Init motors
        motor = new TalonFX(IntakeConstants.motorID);

        // configure motor
        MotorConfigs.configureMotor(motor);
    }

    public void stopIntake() {
        motor.stopMotor();
    }

    public void intake(double power) {
        motor.setControl(new DutyCycleOut(power));
    }

    @Override
    public void periodic() {
        // Nothing needed here
    }
}
