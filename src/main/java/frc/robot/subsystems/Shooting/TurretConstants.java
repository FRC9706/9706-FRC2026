package frc.robot.subsystems.Shooting;

import frc.robot.util.Alliance.AllianceUtils;

public class TurretConstants {
    // Motor IDs
    public static final int firingMotor = 1;
    public static final int rotationMotor = 3;
    public static final int pitchMotor = 3;

    // CAN coder IDs
    public static final int rotationCanCoder = 1;
    public static final int pitchCanCoder = 2;

    // Rotational motor variables
    public static final double gearRatio = 1/135;

    // Rotational motor settings
    public static final double roamSpeed = 0.15;
    public static final double maxRotPower = 0.5;
    public static final double turretDegLim = 50;

    public static final double[] kRotPID = {
      0.04, // kP
      0, // kI
      0.1  // kD
    };

    public static final double kTrackingP = 0.25;

    // Pitch motor settings
    public static final double[] kPitchPID = {
      0.25, // kP%
      0, // kI
      0, // kD
    };

    public static class Limelight {
      public static class Tags {
        // This is the Height of the TAGS (target) from the floor
        public static final double targetH = 1;

        // Dead bands
        public static final double txDeadbandDeg = 0.5;
        public static final double tvDebonceTime = 0.2;

        /* 
          ALL of the following directions are based on as 
          if you where looking at the hub from the front

          Side Note: They are ordered in whicever one is closest
          to the edge
        */
        public static final int[][] blueHubTags = {
          {25, 26}, // Front face
          {24, 21}, // Left face
          {19, 20}, // Back face
          {27, 18} // Right face
        };

        public static final int[][] redHubTags = {
          {9, 10}, // Front face
          {8, 5}, // Left face
          {3, 4}, // Back face
          {11, 2} // Right face
        };

        public static int[][] getAprilTags() {
          AllianceUtils.Alliance alliance = AllianceUtils.getCurrentAlliance(); 
          return switch (alliance) {
            case Blue -> blueHubTags;
            case Red -> redHubTags;
            case Unknown -> new int[0][0];
          };
        }
      }

    // Turret Limelight variables
    /*  
      CRITICAL NOTE: when talking about horizontal/veritcal offsets 
        I am speaking in a 2D manner viewing from the top of the robot
        (horizontal: left & right, vertical foward & back)
    */

    // offset X is the horiztonal offset of the limelight from the center of the turret
    public static final double offsetX = 10;
    // offset Y is the vertical offset of the limelight from the center of thr robot 
    public static final double offsetY = 10;
    // offset H is the height of the limelight's lense to the ground 
    public static final double offsetH = 10;

    /*
      These are the the offset angle measurements for how the limelight is mounted 
      They are in degrees

      ROLL: similar to the motion of a barrel roll
      PITCH: similar to the motion of nodding yes
      YAW: similar to the motion of shaking your head no
    */
    public static final double offsetRoll = 1;
    public static final double offsetPitch = 1;
    public static final double offsetYaw = 1;
    }
}
