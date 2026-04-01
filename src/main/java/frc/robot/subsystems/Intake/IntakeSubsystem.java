package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Networking.DynamicInputs;
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
        
        extMotors = new TalonFX[] {
            new TalonFX(IntakeConstants.extMotorIDs[0]),
            new TalonFX(IntakeConstants.extMotorIDs[1])
        };

        extMotors[0].getConfigurator().apply(IntakeConstants.extConfig);
        extMotors[1].getConfigurator().apply(IntakeConstants.extConfig2);

        for (TalonFX motor : extMotors) {
            motor.setPosition(0);
        }

        // auto dynamic pid tuning
        DynamicInputs.autoTalonFXMotionMagicPID(
            "Intake/Ext Motors[0]", 
            IntakeConstants.extConfig, 
            extMotors[0]
        );

        DynamicInputs.autoTalonFXMotionMagicPID(
            "Intake/Ext Motors[1]", 
            IntakeConstants.extConfig2, 
            extMotors[1]
        );
    }
    public void setExtPos(double pos) {
        extRequest = new MotionMagicVoltage(pos);
    }

    public void setRollerDutyCycle(double dutyCycle) {
        rollerRequest = new DutyCycleOut(dutyCycle);
    }

    public Command extend() {
        return this.runOnce(() -> {
            setExtPos(IntakeConstants.extendedPos);
            setRollerDutyCycle(1);
        });
    }

    public Command retract() {
        return this.runOnce(() -> {
            setExtPos(IntakeConstants.retractedPos);
            setRollerDutyCycle(0);
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
        for (TalonFX motor : extMotors) {
            motor.setControl(extRequest);
        }

        rollerMotor.setControl(rollerRequest);
    }
}
