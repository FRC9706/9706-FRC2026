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
    public Command setRollers(double dutyCycle) {
        return this.runOnce(() ->
            intake.setRollerDutyCycle(dutyCycle)
        );
    }
    public Command stopRollers() {
        return this.runOnce(() ->
            intake.setRollerDutyCycle(0)
        );
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
    public Command setFloorOuttake(double dutyCycle) {
        return this.runOnce(() ->
            floor.setFloorDutyCycle(dutyCycle)
        );
    }

    public Command stopFloor() {
        return this.runOnce(() ->
            floor.setFloorDutyCycle(0)
        );
    }

    // Combined
    public Command intakeFuel(double floorDutyCycle, double rollerDutyCycle) {
        return Commands.sequence(
            Commands.run(() -> intake.setExtPos(IntakeConstants.extendedPos))
                .until(() -> intake.isExtPosReached()),
            //Commands.runOnce(() -> floor.setFloorDutyCycle(-floorDutyCycle)),
            Commands.runOnce(() -> intake.setRollerDutyCycle(rollerDutyCycle))
        );
    }

    public Command endIntakeFuel() {
        return Commands.sequence(
            Commands.runOnce(() -> floor.setFloorDutyCycle(0)),
            Commands.runOnce(() -> intake.setRollerDutyCycle(0))
        );
    }

    public Command outtakeFuel(double floorDutyCycle, double rollerDutyCycle) {
        return Commands.sequence(
            Commands.run(() -> intake.setExtPos(IntakeConstants.extendedPos))
                .until(() -> intake.isExtPosReached()),
            //Commands.runOnce(() -> floor.setFloorDutyCycle(floorDutyCycle)),
            Commands.runOnce(() -> intake.setRollerDutyCycle(-rollerDutyCycle))
        );
    }

    public Command endOuttakeFuel() {
        return Commands.sequence(
            Commands.runOnce(() -> floor.setFloorDutyCycle(0)),
            Commands.runOnce(() -> intake.setRollerDutyCycle(0))
        );
    }
}
