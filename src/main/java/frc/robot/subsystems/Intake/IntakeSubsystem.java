package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class IntakeSubsystem extends SubsystemBase {

    // init motor variables
    @Getter private TalonFX rollerMotor;
    @Getter private TalonFX[] extMotors;

    private MotionMagicVoltage extRequest = new MotionMagicVoltage(0);
    private DutyCycleOut rollerRequest = new DutyCycleOut(0);

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);
        rollerMotor.getConfigurator().apply(IntakeConstants.intakeConfig);
        
        extMotors = new TalonFX[2];
        extMotors[0] = new TalonFX(IntakeConstants.extMotorIDs[0]);
        extMotors[1] = new TalonFX(IntakeConstants.extMotorIDs[1]);

        extMotors[0].getConfigurator().apply(IntakeConstants.extConfig);
        extMotors[1].getConfigurator().apply(IntakeConstants.extConfig2);
    }

    public void setExtPos(double pos) {
        extRequest = new MotionMagicVoltage(pos);
    }

    public void setRollerSpeed(double dutyCycle) {
        rollerRequest = new DutyCycleOut(dutyCycle);
    }

    public Command extend() {
        return this.runOnce(() -> {
            setExtPos(IntakeConstants.extendedPos);
            setRollerSpeed(1);
        });
    }

    public Command retract() {
        return this.runOnce(() -> {
            setExtPos(IntakeConstants.retractedPos);
            setRollerSpeed(0);
        });
    }
    
    public Command wiggle() {
        return Commands.repeatingSequence(
            Commands.runOnce(() -> setExtPos(IntakeConstants.wiggleRange[0])),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setExtPos(IntakeConstants.wiggleRange[1])),
            Commands.waitSeconds(0.5)
        );
    }

    @Override
    public void periodic() {
        extMotors[0].setControl(extRequest);
        extMotors[1].setControl(extRequest);

        rollerMotor.setControl(rollerRequest);
    }
}
