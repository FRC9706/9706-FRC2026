package frc.robot.subsystems.Hopper;

public class HopperConstants {
    // Motor IDs
    public static int[] hopperMotors = {15, 16};

    // Motor Variables
    public static int maxVoltage = 10;

    // Motor positions
    public static double retractedPos = 0;
    public static double extendedPos = 3;
    public static double posTolerance = 1;
    public static double[] wiggleRange = {9,7};

    public static final double[] kPID = {
      10, // kP
      0, // kI
      0,  // kD
      0.25, // kS
      0.10, // kV
      0.01, // kA
      40, // Motion Magic cruise velocity
      120, // Motion Magic acceleration
      1600 // Motion Magic jerk
    };
}
