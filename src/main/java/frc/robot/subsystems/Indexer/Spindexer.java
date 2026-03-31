package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Spindexer extends SubsystemBase{
    // Hardware
    TalonFX motor;

    private DutyCycleOut request = new DutyCycleOut(0);

    public Spindexer() {
        // Intialize Motors
        motor = new TalonFX(SpindexerConstants.motorID);

        // Configure motors
        motor.getConfigurator().apply(SpindexerConstants.config);
    }
    
    public Command setDutyCycle(double dutyCycle) {
        return this.runOnce(() ->
            request = new DutyCycleOut(dutyCycle)
        );    
    }

    @Override
    public void periodic() {
        motor.setControl(request);
    }
}
