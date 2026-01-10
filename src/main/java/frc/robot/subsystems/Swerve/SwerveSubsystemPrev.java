package frc.robot.subsystems.Swerve;
// package frc.robot.subsystems.Swerve;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import java.io.File;
// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Timer;
// import swervelib.parser.SwerveDriveConfiguration;
// import swervelib.parser.SwerveParser;
// import swervelib.SwerveController;
// import swervelib.SwerveDrive;
// import swervelib.SwerveInputStream;
// import swervelib.math.SwerveMath;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.util.Units.*;
// import swervelib.telemetry.SwerveDriveTelemetry;
// import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// public class SwerveSubsystem extends SubsystemBase {
//   /** Creates a new SwerveSubsystem. */
//   File directory = new File(Filesystem.getDeployDirectory(),"swerve");
//   SwerveDrive swerveDrive;

//   public SwerveSubsystem() {
//     SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

//     try
//     {
//       swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Drivetrain.maxSpeed);
//     } catch (Exception e)
//     {
//       throw new RuntimeException(e);
//     }
//   }


//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   // ========== Methods ==========
//   public SwerveDrive getSwerveDrive() {
//     return swerveDrive;
//   }

//   public void driveFieldOriented(ChassisSpeeds velocity) {
//     swerveDrive.driveFieldOriented(velocity);
//   }

//   public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
//     return run(() -> {
//       swerveDrive.driveFieldOriented(velocity.get());
//     });
//   }

//   public Rotation2d getHeading()
//   {
//     return getPose().getRotation();
//   }

//     /**
//    * Command to drive the robot using translative values and heading as angular velocity.
//    *
//    * @param translationX     Translation in the X direction. Cubed for smoother controls.
//    * @param translationY     Translation in the Y direction. Cubed for smoother controls.
//    * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
//    * @return Drive command.
//    */

//   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
//   {
//     return run(() -> {
//       swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
//                             translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
//                             translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
//                         Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
//                         true,
//                         false);
//     });
//   }

//     /**
//    * Command to drive the robot using translative values and heading as a setpoint.
//    *
//    * @param translationX Translation in the X direction. Cubed for smoother controls.
//    * @param translationY Translation in the Y direction. Cubed for smoother controls.
//    * @param headingX     Heading X to calculate angle of the joystick.
//    * @param headingY     Heading Y to calculate angle of the joystick.
//    * @return Drive command.
//    */

//   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
// {
// // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
//   return run(() -> {

//     Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
//     translationY.getAsDouble()), 0.8);

//     // Make the robot move
//     driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
//       scaledInputs.getX(), scaledInputs.getY(),
//       headingX.getAsDouble(),
//       headingY.getAsDouble(),
//       swerveDrive.getOdometryHeading().getRadians(),
//       swerveDrive.getMaximumChassisVelocity()));
//   });

// }

//   /**
//    * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
//    * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
//    * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
//    *
//    * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
//    *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
//    *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
//    *                      (field North) and positive y is torwards the left wall when looking through the driver station
//    *                      glass (field West).
//    * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
//    *                      relativity.
//    * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
//    */

// public void drive(Translation2d translation, double rotation, boolean fieldRelative)
// {
//   swerveDrive.drive(translation,
//                     rotation,
//                     fieldRelative,
//                     false); // Open loop is disabled since it shouldn't be used most of the time.
// }

//   /**
//    * Get the swerve drive kinematics object.
//    *
//    * @return {@link SwerveDriveKinematics} of the swerve drive.
//    */
//   public SwerveDriveKinematics getKinematics()
//   {
//     return swerveDrive.kinematics;
//   }

//   /**
//    * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
//    * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
//    * keep working.
//    *
//    * @param initialHolonomicPose The pose to set the odometry to
//    */
//   public void resetOdometry(Pose2d initialHolonomicPose)
//   {
//     swerveDrive.resetOdometry(initialHolonomicPose);
//   }

//   /**
//    * Gets the current pose (position and rotation) of the robot, as reported by odometry.
//    *
//    * @return The robot's pose
//    */
//   public Pose2d getPose()
//   {
//     return swerveDrive.getPose();
//   }

//   /**
//    * Set chassis speeds with closed-loop velocity control.
//    *
//    * @param chassisSpeeds Chassis Speeds to set.
//    */
//   public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
//   {
//     swerveDrive.setChassisSpeeds(chassisSpeeds);
//   }

//   /**
//    * Post the trajectory to the field.
//    *
//    * @param trajectory The trajectory to post.
//    */
//   public void postTrajectory(Trajectory trajectory)
//   {
//     swerveDrive.postTrajectory(trajectory);
//   }

//   /**
//    * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
//    */
//   public void zeroGyro()
//   {
//     swerveDrive.zeroGyro();
//   }

//   /**
//    * Checks if the alliance is red, defaults to false if alliance isn't available.
//    *
//    * @return true if the red alliance, false if blue. Defaults to false if none is available.
//    */
//   private boolean isRedAlliance()
//   {
//     var alliance = DriverStation.getAlliance();
//     return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
//   }

//   /**
//    * This will zero (calibrate) the robot to assume the current position is facing forward
//    * <p>
//    * If red alliance rotate the robot 180 after the drviebase zero command
//    */
//   public void zeroGyroWithAlliance()
//   {
//     if (isRedAlliance())
//     {
//       zeroGyro();
//       //Set the pose 180 degrees
//       resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
//     } else
//     {
//       zeroGyro();
//     }
//   }

//   /**
//    * Sets the drive motors to brake/coast mode.
//    *
//    * @param brake True to set motors to brake mode, false for coast.
//    */
//   public void setMotorBrake(boolean brake)
//   {
//     swerveDrive.setMotorIdleMode(brake);
//   }

//     /**
//    * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
//    * 90deg.
//    *
//    * @param xInput X joystick input for the robot to move in the X direction.
//    * @param yInput Y joystick input for the robot to move in the Y direction.
//    * @param angle  The angle in as a {@link Rotation2d}.
//    * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
//    */
//   public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
//   {
//     Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

//     return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
//                                                         scaledInputs.getY(),
//                                                         angle.getRadians(),
//                                                         getHeading().getRadians(),
//                                                         Constants.Drivetrain.maxSpeed);
//   }

//   /**
//    * Gets the current field-relative velocity (x, y and omega) of the robot
//    *
//    * @return A ChassisSpeeds object of the current field-relative velocity
//    */
//   public ChassisSpeeds getFieldVelocity()
//   {
//     return swerveDrive.getFieldVelocity();
//   }

//   /**
//    * Gets the current velocity (x, y and omega) of the robot
//    *
//    * @return A {@link ChassisSpeeds} object of the current velocity
//    */
//   public ChassisSpeeds getRobotVelocity()
//   {
//     return swerveDrive.getRobotVelocity();
//   }

//   /**
//    * Get the {@link SwerveController} in the swerve drive.
//    *
//    * @return {@link SwerveController} from the {@link SwerveDrive}.
//    */
//   public SwerveController getSwerveController()
//   {
//     return swerveDrive.swerveController;
//   }

//   /**
//    * Get the {@link SwerveDriveConfiguration} object.
//    *
//    * @return The {@link SwerveDriveConfiguration} fpr the current drive.
//    */
//   public SwerveDriveConfiguration getSwerveDriveConfiguration()
//   {
//     return swerveDrive.swerveDriveConfiguration;
//   }

//   /**
//    * Lock the swerve drive to prevent it from moving.
//    */
//   public void lock()
//   {
//     swerveDrive.lockPose();
//   }

//   /**
//    * Gets the current pitch angle of the robot, as reported by the imu.
//    *
//    * @return The heading as a {@link Rotation2d} angle
//    */
//   public Rotation2d getPitch()
//   {
//     return swerveDrive.getPitch();
//   }



//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }
