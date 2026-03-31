package frc.robot.subsystems.Hopper;

public class HopperConstants {
    // Motor IDs
    public static final int[] hopperMotors = {15, 16};

    // Motor Variables
    public static final int maxVoltage = 10;
    public static final int supplyLimit = 25;
    public static final int statorLimit = 25;

    // Motor positions
    public static final double retractedPos = 0;
    public static final double extendedPos = 3;
    public static final double posTolerance = 1;
    public static final double[] wiggleRange = {3,2};

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
