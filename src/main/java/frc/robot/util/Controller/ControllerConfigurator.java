package frc.robot.util.Controller;

// import java.lang.Thread.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.SwerveStreamType;
import frc.robot.util.Networking.DynamicInputs;
import frc.robot.util.Networking.DynamicInputs.dynamicNum;

public class ControllerConfigurator {
    private static ControllerConfigurator instance;  // Singleton pattern
    
    public static ControllerConfigurator getInstance() {
        if (instance == null) {
            instance = new ControllerConfigurator();
        }
        return instance;
    }

    // Temp tunnable num for RPM
    dynamicNum motorRPM;

    public void configureControllerTL(RobotContainer container) {


    // ====================
    // Button bindings
    // ====================

    // Move foward for one second
    // container.getDriverController().y().whileTrue(
    //     Commands.run(() -> container.getDrivebase().drive(new ChassisSpeeds(0.3, 0, 0)))
    // );

    container.getDriverController().y().whileTrue(
        Commands.run(() -> container.getTurretBetaInst().resetRotEncoderPositons())
    );

    container.getDriverController().x().whileTrue(
        Commands.run(() -> container.getTurretBetaInst().smartMoveToHub())
    );

    // Reset Gyro/Oreint robot to current facing position
    container.getDriverController().a().onTrue(
        Commands.sequence(
            Commands.runOnce(container.getDrivebase()::zeroGyroAndSyncHeading),
            Commands.runOnce(container.getDrivebase()::zeroGyroWithAlliance)
        )
    );

    container.getDriverController().b().whileTrue(
        Commands.run(() -> container.getTurretBetaInst().smartMoveTurretToPos(-190.0/360.0))
    );

    motorRPM = DynamicInputs.number("Turret/ShootRPM", 3400);

    container.getDriverController().leftBumper().onTrue(
        Commands.runOnce(() -> container.getTurretBetaInst().shoot(motorRPM.get())))
         .onFalse(Commands.runOnce(() -> container.getTurretBetaInst().stopShootMotors()));

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
