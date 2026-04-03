package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Networking.DynamicInputs;
import lombok.Getter;

public class IntakeSubsystem extends SubsystemBase {

    // init motor variables
    @Getter private TalonFX rollerMotor;
    @Getter private TalonFX[] extMotors;

    @Getter private MotionMagicVoltage extRequest = new MotionMagicVoltage(0);
    @Getter private DutyCycleOut rollerRequest = new DutyCycleOut(0);

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);
        rollerMotor.getConfigurator().apply(IntakeConstants.intakeConfig);
        
        extMotors = new TalonFX[] {
            new TalonFX(IntakeConstants.extMotorIDs[0]),
            new TalonFX(IntakeConstants.extMotorIDs[1])
        };

        extMotors[0].getConfigurator().apply(IntakeConstants.extConfig);
        extMotors[1].getConfigurator().apply(IntakeConstants.extConfig2);

        extMotors[0].setPosition(0);
        extMotors[1].setPosition(0);

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

    public double getExtMotorsPos(int index) {
        return extMotors[index].getPosition().getValueAsDouble();
    }

    public void setExtPos(double pos) {
        extRequest = new MotionMagicVoltage(pos);
    }

    public boolean isExtPosReached() {
        if (
            (getExtMotorsPos(0) - extRequest.Position) <= IntakeConstants.extPosTolerance
            &&
            (getExtMotorsPos(1) - extRequest.Position) <= IntakeConstants.extPosTolerance
        ) {
            return true;
        } else {
            return false;
        }
    }

    public void setRollerDutyCycle(double dutyCycle) {
        rollerRequest = new DutyCycleOut(dutyCycle);
    }

    @Override
    public void periodic() {
        extMotors[0].setControl(extRequest);
        extMotors[1].setControl(extRequest);

        rollerMotor.setControl(rollerRequest);
    }
}
