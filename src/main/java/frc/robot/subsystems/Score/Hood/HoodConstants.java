package frc.robot.subsystems.Score.Hood;

public class HoodConstants {
    // Motor ID
    public static final int hoodMotorID = 12;

    // Motor Variables
    public static final int maxVoltage = 10;
    public static final double posTolerance = 0.1;

    public static final double[] hoodPID = {
      0, // kP
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
