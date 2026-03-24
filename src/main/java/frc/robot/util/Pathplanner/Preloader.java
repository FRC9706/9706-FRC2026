package frc.robot.util.Pathplanner;

import java.util.function.Function;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command; // Import the Command class
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.Hopper.HopperControl;
import frc.robot.commands.Score.AutoScore;
import frc.robot.subsystems.Hopper.Hopper.hopperState;

public class Preloader {
    // Private constructor to prevent instantiation (since this is a utility)
    private Preloader() {}

    /**
     * A static container class to hold all of the preloaded PathPlanner Commands.
     */
    public static class preloadedAutos {
        // Declare the fields as static so they can be initialized in the static preload method.
        // The type should be PathPlannerAuto, which is a Command.
        public static Command rigthAuto; 
        public static Command middleAuto;
        public static Command leftAuto;
    }

    /**
     * Preloads autos; preventing delays when automous is activated.
     * @implNote Assure that your autos are configured in the {@code preloadedAutos} class & in the function
     * @implNote PathPlannerAuto will load the path file with the given name (e.g., "autohOne.auto")
     * @implNote Make sure named commands are setup for said path.
     */
    public static void preloadsAutos() {
        // Assign the new PathPlannerAuto commands to the static fields.
        // NOTE: PathPlannerAuto will load the path file with the given name (e.g., "autohOne.auto")
        // It will also automatically load ALL referenced .path files inside the auto.
        preloadedAutos.rigthAuto = new PathPlannerAuto("rightAuto");
        preloadedAutos.middleAuto = new PathPlannerAuto("middleAuto");
        preloadedAutos.leftAuto = new PathPlannerAuto("leftAuto");
        
    
        System.out.println("PathPlanner paths have been preloaded!");
    }
    /**
     * Reigsters named commands for any given auto
     * @param auto The auto whose named commands to register.
     * @implNote Assure that the auto you are passing in is handled in the function
     */
    public static void registerNamedCmds(String auto, RobotContainer container) {
        Function<hopperState, Command> hopperCommand = state -> new HopperControl(container.getHopperInst(), state);
        switch(auto) {
            case "rightAuto":
                NamedCommands.clearAll();
                NamedCommands.registerCommand("autoScoreNoIntake", 
                    new AutoScore(container.getTurretBetaInst(), container.getSpindexerInst(), container.getIntakeInst(), 
                    hopperCommand, 
                    true, false)
                );
            break;

            case "centerAuto":
                NamedCommands.clearAll();
                NamedCommands.registerCommand("is", null);
            break;

            case "leftAuto": 
                NamedCommands.clearAll();
                NamedCommands.registerCommand("ben", null);
            break;
        }
    }
    

    /**
     * Utility method to retrieve a preloaded command.
     * @param pathName The name of the path (e.g., "pathOne").
     * @return The preloaded PathPlannerAuto Command.
     */
    public static Command getpath(String pathName) {
        return switch (pathName) {
            case "rightAuto" -> preloadedAutos.rigthAuto;
            case "middleAuto" -> preloadedAutos.middleAuto;
            case "leftAuto" -> preloadedAutos.leftAuto;
            default -> {
                // Handle case where path is not found (e.g., return an empty command or throw error)
                System.err.println("Requested PathPlanner path not preloaded: " + pathName);
                yield Commands.none(); // Return no command
            }
        };
    }
}