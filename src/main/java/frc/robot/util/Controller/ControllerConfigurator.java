package frc.robot.util.Controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SwerveStreamType;
import frc.robot.subsystems.Shooting.Turret.state;

public class ControllerConfigurator {
    private static ControllerConfigurator instance;  // Singleton pattern
    
    public static ControllerConfigurator getInstance() {
        if (instance == null) {
            instance = new ControllerConfigurator();
        }
        return instance;
    }

    public void configureControllerTL(RobotContainer container) {


    // ====================
    // Button bindings
    // ====================

    // Reset Gyro/Oreint robot to current facing position
    container.getDriverController().a().onTrue(
        Commands.sequence(
            Commands.runOnce(container.getDrivebase()::zeroGyroAndSyncHeading),
            Commands.runOnce(container.getDrivebase()::zeroGyroWithAlliance)
        )
    );

    // Move foward for one second
    container.getDriverController().y().whileTrue(
        Commands.run(() -> container.getDrivebase().drive(new ChassisSpeeds(0.3, 0, 0)))
    );

    container.getDriverController().x().onTrue(
        Commands.runOnce(() -> {
            if (container.turretInstance.getState() == state.Idle) {
                container.turretInstance.setState(state.TagFinding);
            } else {
                container.turretInstance.setState(state.Idle);
            }
        })
    );

  }

    public static void configureControllerSim(RobotContainer container) {

      container.getDriverController().start().onTrue(Commands.runOnce(() -> container.getDrivebase().resetOdometry(new Pose2d(3, 3, new Rotation2d()))));

      container.getDriverController().button(1).whileTrue(container.getDrivebase().sysIdDriveMotorCommand());

      container.getDriverController().button(2).whileTrue(Commands.runEnd(() -> container.getSwerveInputStream(SwerveStreamType.DirectAngleKeyboard).driveToPoseEnabled(true),
      () -> container.getSwerveInputStream(SwerveStreamType.DirectAngleKeyboard).driveToPoseEnabled(false)));
      //  driverXbox.b().whileTrue(
      //      drivebase.driveToPose(
      //          new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                          );
    }

    public static void configureControllerTest(RobotContainer container) {}
}
