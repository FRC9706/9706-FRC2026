package frc.robot.subsystems.Floor;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class FloorSubsystem extends SubsystemBase {
    // Init variables
    @Getter private final TalonFX floorMotor;

    private DutyCycleOut mRequest = new DutyCycleOut(0);

    public FloorSubsystem() {
        floorMotor = new TalonFX(FloorConstants.floorMotorID);
    }

    public void setFloorDutyCycle(double dutyCycle) {
        mRequest = new DutyCycleOut(dutyCycle);
    }

    public Command toggleFloorOuttake() {
        return this.runOnce(() -> {
            if (mRequest.Output != 1) {
                setFloorDutyCycle(0.20);
            } else {
                setFloorDutyCycle(0);
            }
        });
    }
    
    @Override
    public void periodic() {
        floorMotor.setControl(mRequest);
    }
}
