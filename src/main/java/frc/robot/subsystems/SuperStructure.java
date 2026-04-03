package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Floor.FloorSubsystem;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class SuperStructure extends SubsystemBase {
    // Subsystems
    private final IntakeSubsystem intake;
    private final FloorSubsystem floor;
    
    public SuperStructure(IntakeSubsystem intakeInput, FloorSubsystem floorInput) {
        intake = intakeInput;
        floor = floorInput;
    }

    // Intake roller commands
    public Command toggleRollerDir() {
        double rollerDutyCycle = 1;

        return this.runOnce(() -> {
            if (intake.getRollerRequest().Output != rollerDutyCycle) {
                intake.setRollerDutyCycle(rollerDutyCycle);
            } else {
                intake.setRollerDutyCycle(-rollerDutyCycle);
            }
        });
    }

    // Intake extender commands
    public Command extendIntake() {
        return this.runOnce(() -> {
            intake.setExtPos(IntakeConstants.extendedPos);
        });
    }

    public Command retractIntake() {
        return this.runOnce(() -> {
            intake.setExtPos(IntakeConstants.retractedPos);
        });
    }
    
    public Command wiggleIntake() {
        return Commands.repeatingSequence(
            Commands.runOnce(() -> intake.setExtPos(IntakeConstants.wiggleRange[0])),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> intake.setExtPos(IntakeConstants.wiggleRange[1])),
            Commands.waitSeconds(0.5)
        );
    }

    // Floor commands
    public Command toggleFloorOuttake() {
        return this.runOnce(() -> {
            double dutyCycle = 0.20;

            if (floor.getFloorRequest().Output != dutyCycle) {
                floor.setFloorDutyCycle(dutyCycle);
            } else {
                floor.setFloorDutyCycle(0);
            }
        });
    }   
}
