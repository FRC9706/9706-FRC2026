package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
public class Spindexer extends SubsystemBase{
    // Hardware
    @Getter TalonFX spindexerMotor;

    private DutyCycleOut request = new DutyCycleOut(0);

    public Spindexer() {
        // Intialize Motors
        spindexerMotor = new TalonFX(SpindexerConstants.motorID);

        // Configure motors
        spindexerMotor.getConfigurator().apply(SpindexerConstants.config);
    }
    
    public Command setDutyCycle(double dutyCycle) {
        return this.runOnce(() ->
            request = new DutyCycleOut(dutyCycle)
        );    
    }

    @Override
    public void periodic() {
        spindexerMotor.setControl(request);
    }
}
