// package frc.robot.commands.Score;

// import java.util.function.Function;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.Indexer.Spindexer;
// import frc.robot.subsystems.Intake.Intake;
// import frc.robot.subsystems.Score.TurretBeta;

// import frc.robot.subsystems.Hopper.Hopper.hopperState;

// public class AutoScore extends Command {
//   // initalize subsystem varibales
//   private final TurretBeta mTurretBeta;
//   private final Spindexer mSpindexer;
//   private final Intake mIntake;

//   // initalize command variables
//   private final Function<hopperState, Command> hopperControlFac;
//     private final Command extHopper;
//     private final Command wiggleHopper;

//   // Initalize all other variables
//   private final boolean runIntake;
//   private final boolean runWiggle;

//   /**
//    * This command will score using the turret (auto lock on & shoot), run the spindexer, and optionally run the intake as well.
//    * @param turretBeta Requires access to the turret beta subsystem to run turret commands.
//    * @param spindexer Requires access to the indexer subsystem to run the indexer.
//    * @param intake Requires access to the intake subsystem to run the intake.
//    * @param hopperCommand Needs access to the hopper command in order to extend, wiggle, or retract the hopper.
//    * @param runIntakeInput Chose whether or not you want to run the intake.
//    * @param runWiggleInput Chose whether or not you want to wiggle the hopper
//    * @throws IllegalArgumentException Will literally blow up the entire code if you don't read the this and mess somethign up to avoid a worse blowing up of the code due to a NPE.
//    * @implNote if you would like run the intake, intialization will automanically extend the hopper.
//    * @implNote DO NOT USE {@code runWiggleInput} IF {@code runIntakeInput} IS TRUE and VICE VERSA; This WILL cuase an {@code IllegalArgumentException}.
//    * @implNote Do NOT pass {@code null} in for {@code hopperCommand}
//    */
//   public AutoScore(
//     TurretBeta turretBeta, Spindexer spindexer, Intake intake,
//     Function<hopperState, Command> hopperCommand,
//     boolean runIntakeInput,
//     boolean runWiggleInput) {
//     // Bind inputs to variables
//     mTurretBeta = turretBeta;
//     mSpindexer = spindexer;
//     mIntake = intake;
//     runIntake = runIntakeInput;
//     runWiggle = runWiggleInput;

//     hopperControlFac = hopperCommand;

//     // Use the hopper control factory to create hopper commands
//     if (hopperControlFac == null) {
//       throw new IllegalArgumentException(
//         "AutoScore: Are we serious bro read the documentation I spent like 30 mins on why did you not pass in a hopper command?"
//       );
//     }
    
//     if (runIntakeInput && runWiggleInput) {
//       throw new IllegalArgumentException(
//         "AutoScore: It literally tells you not to have runIntake and runWiggle both be true???"
//       );
//     }

//     extHopper = hopperControlFac.apply(hopperState.EXTENDED);
//     wiggleHopper = hopperControlFac.apply(hopperState.WIGGLE);

//     // Declare subsystem dependencies.
//     addRequirements(turretBeta);
//     addRequirements(spindexer);
//     addRequirements(intake);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (runIntake && (extHopper != null)) {
//         System.out.println("AutoScore initalization: extending hopper becuase runIntake is: " + runIntake);
//         CommandScheduler.getInstance().schedule(
//           extHopper
//         );
//       }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//   }
// }
