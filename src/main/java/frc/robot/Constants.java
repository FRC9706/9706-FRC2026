// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

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
    public static final int firingMotor = 1;
    public static final int turnMotor = 2;
    public static final int angleMotor = 3;

  }

  public static class Drivetrain {


    // -------------------------------
    //        Supply limits
    // -------------------------------
    public static final double kStatorCurrent = 80;
    public static final double kSupplyCurrent = 40;
    public static final double kStatorDriveCurrent = 170;


    // -------------------------------
    //        Gear ratios
    // -------------------------------
    public static final double gearRatio = 1;
    public static final double kDriveGearRatio = 6.48;
    public static final double kSteerGearRatio = 12.1;
    public static final double kCoupleRatio = 54.0 / 12.0;


    // -------------------------------
    //       Frame dimensions
    // -------------------------------
    public static final double kFrameLengthInches = 29.0;
    public static final double kFrameWidthInches = 29.0;


    // -------------------------------
    //       Wheel dimensions
    // -------------------------------
    public static final double kWheelRadiusInches = 2.167;


    // -------------------------------
    //          Gyro / IMU
    // -------------------------------
    public static final int kPigeonID = 13;


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
    public static final boolean enableHeadingCorrection = true;  // Should only be used in angle control mode
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
    public static final double translationP = 20;
    public static final double translationI = 0.0;
    public static final double translationD = 0.6;


    // Rotation PID tuning (radians)
    public static final double rotationP = 20;
    public static final double rotationI = 0.0;
    public static final double rotationD = 0.6;


    // -------------------------------
    //    Path constraints variables
    // -------------------------------
    public static final double maxAccel = 4.0;                 // meters per second squared
    public static final double goalEndVelocity = 0.0;          // Target stop speed in m/s


    // -------------------------------
    //     Feedforward coefficients
    // -------------------------------
    public static final double kS = 0.1;  // Static gain
    public static final double kV = 2.2;  // Velocity gain
    public static final double kA = 0.3;  // Acceleration gain


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
      public static final int kEncoderID = 12;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kFrontLeftXPos;
      public static final double kYPos = Drivetrain.kFrontLeftYPos;
      public static final double kEncoderOffsetRotations = -0.321 + 0.25;
    }


    public static class FrontRightModule {
      // --- CAN IDs ---
      public static final int kDriveMotorID = 1;
      public static final int kSteerMotorID = 2;
      public static final int kEncoderID = 9;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kFrontRightXPos;
      public static final double kYPos = Drivetrain.kFrontRightYPos;
      public static final double kEncoderOffsetRotations = -0.242 + (1.0 / 8.0) + 0.5;
    }


    public static class BackLeftModule {
      // --- CAN IDs ---
      public static final int kDriveMotorID = 5;
      public static final int kSteerMotorID = 6;
      public static final int kEncoderID = 11;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kBackLeftXPos;
      public static final double kYPos = Drivetrain.kBackLeftYPos;
      public static final double kEncoderOffsetRotations = -0.500 - (3.0 / 8.0) + 0.5;
    }


    public static class BackRightModule {
      // --- CAN IDs ---
      public static final int kDriveMotorID = 3;
      public static final int kSteerMotorID = 4;
      public static final int kEncoderID = 10;


      // --- Motor/Encoder Inversion ---
      public static final boolean kSteerMotorInverted = true;
      public static final boolean kEncoderInverted = false;


      // --- Module Offsets ---
      public static final double kXPos = Drivetrain.kBackRightXPos;
      public static final double kYPos = Drivetrain.kBackRightYPos;
      public static final double kEncoderOffsetRotations = -0.3459 - (1.0 / 8.0) + 0.5;
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
    public static final double profiledKp = 5;
    public static final double profiledKi = 0;
    public static final double profiledKd = 0;


    // --- Velocity --- 
    public static final double maxVel = 5;
    public static final double maxAccel = 2;
  }
}
