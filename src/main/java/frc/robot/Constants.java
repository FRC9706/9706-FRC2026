// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import frc.robot.util.Alliance.AllianceUtils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class Controller {
    public static final int kDriverControllerPort = 1;
    public static final double deadband = 0.05;
    public static final double scaleTranslation = 0.1;
    public static final double scaleRotation = -0.25;
  }

  public static class Turret {
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

  public class limelights {
    public static final String limelightTurretIP = "172.29.0.1";
    public static final String limelightFrontIP = "172.29.1.1";
    public static final String limelightRightIP = "172.29.2.1";
    public static final String limelightLeftIP = "172.29.3.1";
  }

  public static class Drivetrain {


    // -------------------------------
    //        Supply limits
    // -------------------------------
    public static final double kStatorCurrent = 40; // Make sure to update in JSON if changed here
    public static final double kSupplyCurrent = 20; // Make sure to update in JSON if changed here
    public static final double kStatorDriveCurrent = 170;


    // -------------------------------
    //        Gear ratios
    // -------------------------------
    public static final double gearRatio = 1;
    public static final double kDriveGearRatio = 6.82; // Make sure to update in JSON if changed here
    public static final double kSteerGearRatio = 12.1; // Make sure to update in JSON if changed here
    public static final double kCoupleRatio = 54.0 / 12.0;


    // -------------------------------
    //       Frame dimensions
    // -------------------------------
    public static final double kFrameLengthInches = 27.5;
    public static final double kFrameWidthInches = 27.5;


    // -------------------------------
    //       Wheel dimensions
    // -------------------------------
    public static final double kWheelRadiusInches = 4;


    // -------------------------------
    //          Gyro / IMU
    // -------------------------------
    public static final int kPigeonID = 9;


    // -------------------------------
    //         Max speeds
    // -------------------------------
    public static final double maxSpeed = 10.0; // meters per second
    public static final double maxAngularVelocity = Math.toRadians(720); // radians per second


    // -------------------------------
    //       Alliance Settings
    // -------------------------------
    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = false;


    // -------------------------------
    //   Telemetry and compensation
    // -------------------------------
    public static final boolean enableTelemetry = true;
    public static final boolean enableHeadingCorrection = false;  // Should only be used in angle control mode
    public static final boolean enableCosineCompensation = false; // Disabled in sim for accuracy
    public static final boolean enableAngularVelocityComp = true; // Skew correction
    public static final double angularVelocityCompCoeff = 0.1;    // Compensation coefficient
    public static final boolean enableEncoderAutoSync = false;    // Auto re-sync of abs encoders
    public static final double encoderAutoSyncInterval = 1.0;     // Sync every 1 second


    // -------------------------------
    //       PathPlanner PID
    // -------------------------------
    public static final boolean enableFeedforward = false; // Enables feedforward assist in PathPlanner control loops


    // Translation PID tuning (meters)
    public static final double translationP = 0;
    public static final double translationI = 0;
    public static final double translationD = 0;


    // Rotation PID tuning (radians)
    public static final double rotationP = 0;
    public static final double rotationI = 0;
    public static final double rotationD = 0;


    // -------------------------------
    //    Path constraints variables
    // -------------------------------
    public static final double maxAccel = 4.0;                 // meters per second squared
    public static final double goalEndVelocity = 0.0;          // Target stop speed in m/s


    // -------------------------------
    //     Feedforward coefficients
    // -------------------------------
    // public static final double kS = 0.1;  // Static gain
    // public static final double kV = 2.2;  // Velocity gain
    // public static final double kA = 0.3;  // Acceleration gain
    // Unused, most likely uneeded/unimportant


    // -------------------------------
    //         SysId test params
    // -------------------------------
    public static final double sysIdRampRate = 3.0;      // V/s
    public static final double sysIdStepVoltage = 5.0;   // V
    public static final double sysIdTestDuration = 3.0;  // seconds


    // -------------------------------
    //   Module location offsets (in)
    // -------------------------------
    // Front left offsets
    public static final double kFrontLeftXPos = (kFrameLengthInches / 2.0) - 2.5;
    public static final double kFrontLeftYPos = (kFrameWidthInches / 2.0) - 2.5;
    // Front right offsets
    public static final double kFrontRightXPos = (kFrameLengthInches / 2.0) - 2.5;
    public static final double kFrontRightYPos = (-kFrameWidthInches / 2.0) + 2.5;
    // Back left offsets
    public static final double kBackLeftXPos = (-kFrameLengthInches / 2.0) + 2.5;
    public static final double kBackLeftYPos = (kFrameWidthInches / 2.0) - 2.5;
    // Back right offsets
    public static final double kBackRightXPos = (-kFrameLengthInches / 2.0) + 2.5;
    public static final double kBackRightYPos = (-kFrameWidthInches / 2.0) + 2.5;


    // -------------------------------
    //        Module Definitions
    // -------------------------------


    public static class FrontLeftModule {
      // --- CAN IDs ---
      public static final int kDriveMotorID = 7;
      public static final int kSteerMotorID = 8;
      public static final int kEncoderID = 4;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kFrontLeftXPos;
      public static final double kYPos = Drivetrain.kFrontLeftYPos;
      public static final double kEncoderOffsetRotations = 32.783203125;
    }


    public static class FrontRightModule {
      // --- CAN IDs ---
      public static final int kDriveMotorID = 1;
      public static final int kSteerMotorID = 2;
      public static final int kEncoderID = 1;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kFrontRightXPos;
      public static final double kYPos = Drivetrain.kFrontRightYPos;
      public static final double kEncoderOffsetRotations = 300.146484375;
    }


    public static class BackLeftModule {
      // --- CAN IDs ---
      public static final int kDriveMotorID = 5;
      public static final int kSteerMotorID = 6;
      public static final int kEncoderID = 3;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kBackLeftXPos;
      public static final double kYPos = Drivetrain.kBackLeftYPos;
      public static final double kEncoderOffsetRotations = 269.560546875;
    }


    public static class BackRightModule {
      // --- CAN IDs ---
      public static final int kDriveMotorID = 3;
      public static final int kSteerMotorID = 4;
      public static final int kEncoderID = 2;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kBackRightXPos;
      public static final double kYPos = Drivetrain.kBackRightYPos;
      public static final double kEncoderOffsetRotations = 222.539062;
    }
  }


  public static class Swerve {
    // -------------------------------
    //     Swerve Subsystem Settings
    // -------------------------------
    public static final boolean blueAllianceDefault = false;
    public static final double startingPoseBlueX = 1.0;
    public static final double startingPoseBlueY = 4.0;
    public static final double startingPoseRedX = 16.0;
    public static final double startingPoseRedY = 4.0;

    // -------------------------------
    //     PathPlanner tuning
    // -------------------------------
    public static final boolean enableFeedforward = Drivetrain.enableFeedforward;
    public static final double translationP = Drivetrain.translationP;
    public static final double translationI = Drivetrain.translationI;
    public static final double translationD = Drivetrain.translationD;
    public static final double rotationP = Drivetrain.rotationP;
    public static final double rotationI = Drivetrain.rotationI;
    public static final double rotationD = Drivetrain.rotationD;

    // -------------------------------
    //     Chassis limits
    // -------------------------------
    public static final double maxSpeed = Drivetrain.maxSpeed;
    public static final double maxAccel = Drivetrain.maxAccel;
    public static final double maxAngularVelocity = Drivetrain.maxAngularVelocity;

    // -------------------------------
    //     Telemetry / Debug Flags
    // -------------------------------
    public static final boolean enableTelemetry = Drivetrain.enableTelemetry;
    public static final boolean headingCorrection = Drivetrain.enableHeadingCorrection;
    public static final boolean cosineCompensation = Drivetrain.enableCosineCompensation;
    public static final boolean angularVelocityComp = Drivetrain.enableAngularVelocityComp;
    public static final double angularVelocityCoeff = Drivetrain.angularVelocityCompCoeff;
    public static final boolean encoderAutoSync = Drivetrain.enableEncoderAutoSync;
    public static final double encoderAutoSyncInterval = Drivetrain.encoderAutoSyncInterval;
  }


  public static class simulation {
    // -------------------------------
    // PID & other drivetrain variables
    // -------------------------------


    // --- PID ---
    public static final double profiledKp = 0;
    public static final double profiledKi = 0;
    public static final double profiledKd = 0;


    // --- Velocity --- 
    public static final double maxVel = 5;
    public static final double maxAccel = 2;
  }
}
