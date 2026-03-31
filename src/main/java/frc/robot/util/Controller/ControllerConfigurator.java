package frc.robot.util.Controller;

// import java.lang.Thread.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import lib.Networking.DynamicInputs.dynamicNum;

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

        // Reset Gyro/Oreint robot to current facing position
        container.getMDriverController().a().onTrue(
            Commands.runOnce(container.getDrivebase()::zeroGyroWithAlliance)
        );

        container.getMDriverController().b().onTrue(
            Commands.run(
                () -> {
                    container.getTurret().targetHub(container.getDrivebase().getPose());
                    container.getHood().targetHub(0);
                    container.getFlywheel().targetHub(0);
                }
            )
        );

        container.getMDriverController().povUp().onTrue(
            Commands.runOnce(() -> container.getTurret().setTurretPosition(Units.degreesToRotations(90)))
        );
        container.getMDriverController().povRight().onTrue(
            Commands.runOnce(() -> container.getTurret().setTurretPosition(Units.degreesToRotations(0)))
        );
                container.getMDriverController().povDown().onTrue(
            Commands.runOnce(() -> container.getTurret().setTurretPosition(Units.degreesToRotations(270)))
        );
        container.getMDriverController().povLeft().onTrue(
            Commands.runOnce(() -> container.getTurret().setTurretPosition(Units.degreesToRotations(180)))
        );

        container.getMDriverController().rightBumper().onTrue(
            container.getIntake().extend()
        );

        container.getMDriverController().leftBumper().onTrue(
            container.getIntake().retract()
        );
   }

    public void configureControllerSim(RobotContainer container) {
        // THIS IS SIMULATION

      container.getMDriverController().start().onTrue(Commands.runOnce(() -> 
        container.getDrivebase().resetOdometry(
            new Pose2d(3, 3, new Rotation2d())
        )
      ));

        // THIS IS SIMULATION

        // THIS IS SIMULATION

        // THIS IS SIMULATION
    }

    public static void configureControllerTest(RobotContainer container) {}
}
