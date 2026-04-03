package frc.robot.subsystems.Floor;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class FloorSubsystem extends SubsystemBase {
    // Init variables
    @Getter private final TalonFX floorMotor;

    @Getter private DutyCycleOut floorRequest = new DutyCycleOut(0);

    public FloorSubsystem() {
        floorMotor = new TalonFX(FloorConstants.floorMotorID);
    }

    public void setFloorDutyCycle(double dutyCycle) {
        floorRequest = new DutyCycleOut(dutyCycle);
    }
    
    @Override
    public void periodic() {
        floorMotor.setControl(floorRequest);
    }
}
