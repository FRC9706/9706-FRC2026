package frc.robot.util.Pathplanner;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command; // Import the Command class
import edu.wpi.first.wpilibj2.command.Commands;

public class Preloader {
    // Private constructor to prevent instantiation (since this is a utility)
    private Preloader() {}

    /**
     * A static container class to hold all of the preloaded PathPlanner Commands.
     */
    public static class preloadedPaths {
        // Declare the fields as static so they can be initialized in the static preload method.
        // The type should be PathPlannerAuto, which is a Command.
        public static Command pathOne; 
        public static Command pathTwo;
    }

    // Preloads all of the desired trajctories
    public static void preloadPaths() {
        // Assign the new PathPlannerAuto commands to the static fields.
        // NOTE: PathPlannerAuto will load the path file with the given name (e.g., "pathOne.path").
        preloadedPaths.pathOne = new PathPlannerAuto("pathOne");
        preloadedPaths.pathTwo = new PathPlannerAuto("pathTwo");
        
    
        System.out.println("PathPlanner paths have been preloaded!");
    }

    /**
     * Utility method to retrieve a preloaded command.
     * @param pathName The name of the path (e.g., "pathOne").
     * @return The preloaded PathPlannerAuto Command.
     */
    public static Command getpath(String pathName) {
        return switch (pathName) {
            case "pathOne" -> preloadedPaths.pathOne;
            case "pathTwo" -> preloadedPaths.pathTwo;
            default -> {
                // Handle case where path is not found (e.g., return an empty command or throw error)
                System.err.println("Requested PathPlanner path not preloaded: " + pathName);
                yield Commands.none(); // Return no command
            }
        };
    }
}