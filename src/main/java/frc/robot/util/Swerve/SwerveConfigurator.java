package frc.robot.util.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import swervelib.SwerveInputStream;


public class SwerveConfigurator {
    // Private constructor to prevent instantiation (since this is a utility)
    private SwerveConfigurator() {} 

    public static class SwerveStreams {
        public SwerveInputStream driveAngularVelocity;
        public SwerveInputStream driveDirectAngle;
        public SwerveInputStream driveRobotOriented;
        public SwerveInputStream driveAngularVelocityKeyboard;
        public SwerveInputStream driveDirectAngleKeyboard;    
    }

    public static SwerveStreams InputStreams(RobotContainer container) {
        SwerveStreams stream = new SwerveStreams();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
        stream.driveAngularVelocity = SwerveInputStream.of(
            container.getDrivebase().getSwerveDrive(),
            () -> container.getDriverController().getLeftY() * -1,
            () -> container.getDriverController().getLeftX() * -1)
            .withControllerRotationAxis(container.getDriverController()::getRightX).scaleRotation(Constants.Controller.scaleRotation)
            .deadband(Constants.Controller.deadband)
            .scaleTranslation(Constants.Controller.scaleTranslation)
            .allianceRelativeControl(true);

 /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
        stream.driveDirectAngle = stream.driveAngularVelocity.copy()
            .withControllerHeadingAxis(container.getDriverController()::getRightX, container.getDriverController()::getRightY)
            .headingWhile(true);

 /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
        stream.driveRobotOriented = stream.driveAngularVelocity.copy()
            .robotRelative(true)
            .allianceRelativeControl(false);

 /**
   * Converts driver input from a keyboard into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
        stream.driveAngularVelocityKeyboard = SwerveInputStream.of(
                container.getDrivebase().getSwerveDrive(),
                () -> -container.getDriverController().getLeftY(),
                () -> -container.getDriverController().getLeftX())
                .withControllerRotationAxis(() -> container.getDriverController().getRawAxis(2)).scaleRotation(Constants.Controller.scaleRotation)
                .deadband(Constants.Controller.deadband)
                .scaleTranslation(Constants.Controller.scaleTranslation)
                .allianceRelativeControl(true);

 /**
   * Clone's the keyboard angular velocity input stream and converts it to a fieldRelative keyboard input stream.
   */
        stream.driveDirectAngleKeyboard = stream.driveAngularVelocityKeyboard.copy()
                .withControllerHeadingAxis(() ->
                  Math.sin(
                    container.getDriverController().getRawAxis(2) * Math.PI) * (Math.PI * 2),
                    () ->
                      Math.cos(
                        container.getDriverController().getRawAxis(2) * Math.PI) * (Math.PI * 2))
                       .headingWhile(true)
                       .translationHeadingOffset(true)
                       .translationHeadingOffset(Rotation2d.fromDegrees(0));
        return stream;
    }
}
