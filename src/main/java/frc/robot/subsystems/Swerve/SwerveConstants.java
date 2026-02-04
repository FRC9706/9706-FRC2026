package frc.robot.subsystems.Swerve;

public class SwerveConstants {

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
      public static final double kXPos = SwerveConstants.kFrontLeftXPos;
      public static final double kYPos = SwerveConstants.kFrontLeftYPos;
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
      public static final double kXPos = SwerveConstants.kFrontRightXPos;
      public static final double kYPos = SwerveConstants.kFrontRightYPos;
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
      public static final double kXPos = SwerveConstants.kBackLeftXPos;
      public static final double kYPos = SwerveConstants.kBackLeftYPos;
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
      public static final double kXPos = SwerveConstants.kBackRightXPos;
      public static final double kYPos = SwerveConstants.kBackRightYPos;
      public static final double kEncoderOffsetRotations = 222.539062;
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
    public static final boolean enableFeedforward = SwerveConstants.enableFeedforward;
    public static final double translationP = SwerveConstants.translationP;
    public static final double translationI = SwerveConstants.translationI;
    public static final double translationD = SwerveConstants.translationD;
    public static final double rotationP = SwerveConstants.rotationP;
    public static final double rotationI = SwerveConstants.rotationI;
    public static final double rotationD = SwerveConstants.rotationD;

    // -------------------------------
    //     Chassis limits
    // -------------------------------
    public static final double maxSpeed = SwerveConstants.maxSpeed;
    public static final double maxAccel = SwerveConstants.maxAccel;
    public static final double maxAngularVelocity = SwerveConstants.maxAngularVelocity;

    // -------------------------------
    //     Telemetry / Debug Flags
    // -------------------------------
    public static final boolean enableTelemetry = SwerveConstants.enableTelemetry;
    public static final boolean headingCorrection = SwerveConstants.enableHeadingCorrection;
    public static final boolean cosineCompensation = SwerveConstants.enableCosineCompensation;
    public static final boolean angularVelocityComp = SwerveConstants.enableAngularVelocityComp;
    public static final double angularVelocityCoeff = SwerveConstants.angularVelocityCompCoeff;
    public static final boolean encoderAutoSync = SwerveConstants.enableEncoderAutoSync;
    public static final double encoderAutoSyncInterval = SwerveConstants.encoderAutoSyncInterval;
  }
}