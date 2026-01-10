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
    public static class preloadedTrajectories {
        // Declare the fields as static so they can be initialized in the static preload method.
        // The type should be PathPlannerAuto, which is a Command.
        public static Command trajectoryOne; 
        public static Command trajectoryTwo;
    }

    // Preloads all of the desired trajctories
    public static void preloadTrajectories() {
        // Assign the new PathPlannerAuto commands to the static fields.
        // NOTE: PathPlannerAuto will load the path file with the given name (e.g., "trajectoryOne.path").
        preloadedTrajectories.trajectoryOne = new PathPlannerAuto("trajectoryOne");
        preloadedTrajectories.trajectoryTwo = new PathPlannerAuto("trajectoryTwo");
        
    
        System.out.println("PathPlanner trajectories have been preloaded!");
    }

    /**
     * Utility method to retrieve a preloaded command.
     * @param trajectoryName The name of the trajectory (e.g., "trajectoryOne").
     * @return The preloaded PathPlannerAuto Command.
     */
    public static Command getTrajectory(String trajectoryName) {
        return switch (trajectoryName) {
            case "trajectoryOne" -> preloadedTrajectories.trajectoryOne;
            case "trajectoryTwo" -> preloadedTrajectories.trajectoryTwo;
            default -> {
                // Handle case where path is not found (e.g., return an empty command or throw error)
                System.err.println("Requested PathPlanner trajectory not preloaded: " + trajectoryName);
                yield Commands.none(); // Return no command
            }
        };
    }
}