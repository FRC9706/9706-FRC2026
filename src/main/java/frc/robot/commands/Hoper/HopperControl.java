package frc.robot.commands.Hoper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.Hopper.hopperState;

public class HopperControl extends Command {
  // initalize subsystem varibales
  private final Hopper mHopper;
  private final hopperState desiredState;

  // initalize all other variables
  private boolean hasReachedGoal;

  /**
   * This command will set the hopper to the given ENUM state
   * @param subsystem Requires access to the hopper subsystem
   * @implNote There is literally no purpose to pass an idle state into this.
   */
  public HopperControl(Hopper hopper, hopperState desiredStateInput) {
    mHopper = hopper;
    desiredState = desiredStateInput;

    // Declare subsystem dependencies.
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (desiredState == hopperState.RETRACTED) {
      mHopper.retractMotors();
    } else if (desiredState == hopperState.EXTENDED) {
      mHopper.extendMotors();
    } else if (desiredState == hopperState.IDLE) {
      System.out.println(
        "Why would you give me an idle state you bafoon just use stopMotors(); if that is what you are looking for"
      );
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return command just incase idle is passed in
    hasReachedGoal = (desiredState == mHopper.getCurrHopperState()) || (desiredState == hopperState.IDLE);

    if (hasReachedGoal) {
      System.out.println("The hopper reached the desired state: " + desiredState.toString());
      System.out.println("Exiting hopper command now");
    }
    return hasReachedGoal;
  }
}
