package frc.robot.commands.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperConstants;
import frc.robot.subsystems.Hopper.Hopper.hopperState;

public class HopperControl extends Command {
  // initalize subsystem varibales
  private final Hopper mHopper;
  private final hopperState desiredState;

  // initalize all other variables
  
  /**
   * This command will set the hopper to the given ENUM state
   * @param subsystem Requires access to the hopper subsystem
   * @implNote Don't pass in an IDLE state, there is literally no purpose to pass an idle state into this.
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
    if (desiredState == hopperState.WIGGLE) {
      System.out.println("The desired state is wiggle; setting hopper state to wiggle");
      mHopper.setHopperState(hopperState.WIGGLE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (desiredState == hopperState.EXTENDED) {
      mHopper.extendMotors();
    } 
      
    if (desiredState == hopperState.RETRACTED) {
      mHopper.retractMotors();
    }
      
    if (desiredState == hopperState.IDLE) {
      System.out.println(
        "Why would you give me an idle state you bafoon just use stopMotors(); if that is what you are looking for"
      );
    }

    if (desiredState == hopperState.WIGGLE) {
      if (!(mHopper.hasReachedPos(HopperConstants.wiggleRange[0]))) {
        mHopper.moveHopperToPos(HopperConstants.wiggleRange[0]);
      } else if (!(mHopper.hasReachedPos(HopperConstants.wiggleRange[1]))) {
        mHopper.moveHopperToPos(HopperConstants.wiggleRange[1]);
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (desiredState == hopperState.WIGGLE) {
      System.out.println("Hopper command was wiggle, exiting command & reseting hopper status to idle.");
      mHopper.setHopperState(hopperState.IDLE);
    } else {
       System.out.println("Exiting hopper command now");
    }
  }
}
