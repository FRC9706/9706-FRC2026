// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.util.Controller.ControllerConfigurator;
import frc.robot.util.Pathplanner.Preloader;
import frc.robot.util.Swerve.SwerveConfigurator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Controller.kDriverControllerPort);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final Limelight limelightInstance = Limelight.getInstance();

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public CommandXboxController getDriverController() {
    return m_driverController;
  }

  //test
  // Enums for the swerve input streams
  public enum SwerveStreamType {
    AngularVelocity,
    DirectAngle,
    RobotOreinted,
    AngularVelocityKeyboard,
    DirectAngleKeyboard
  }

  // Method to get swerve input streams for classes that cannot access the protected variables
  public SwerveInputStream getSwerveInputStream(SwerveStreamType type) {
    switch (type) {
        case AngularVelocity:
            return driveAngularVelocity;
        case DirectAngle:
            return driveDirectAngle;
        case RobotOreinted:
            return driveRobotOriented;
        case AngularVelocityKeyboard:
            return driveAngularVelocityKeyboard;
        case DirectAngleKeyboard:
            return driveDirectAngleKeyboard;
        default:
            // Handle error case, return null or throw an exception
            System.err.println("Requested SwerveInputStream type not found!");
            return null;
    }
  }

  // Intiate the input streams
  protected SwerveInputStream driveAngularVelocity;
  protected SwerveInputStream driveDirectAngle;
  protected SwerveInputStream driveRobotOriented;
  protected SwerveInputStream driveAngularVelocityKeyboard;
  protected SwerveInputStream driveDirectAngleKeyboard;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    var streams = SwerveConfigurator.InputStreams(this);
    driveAngularVelocity = streams.driveAngularVelocity;
    driveDirectAngle = streams.driveDirectAngle;
    driveRobotOriented = streams.driveRobotOriented;
    driveAngularVelocityKeyboard = streams.driveAngularVelocityKeyboard;
    driveDirectAngleKeyboard = streams.driveDirectAngleKeyboard;

    // Configure the trigger bindings
    setupBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Configure path planner & Load Trajectories
    drivebase.setupPathPlanner();

    // Preload any trajectories in Path Planner
    Preloader.preloadTrajectories();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void setupBindings() {
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    // Set the driving method depending on mode (Real or Simulation)
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      ControllerConfigurator.configureControllerTL(this);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
        new ProfiledPIDController(Constants.simulation.profiledKd, Constants.simulation.profiledKi, Constants.simulation.profiledKd,
        new Constraints(Constants.simulation.maxVel, Constants.simulation.maxAccel)),
        new ProfiledPIDController(Constants.simulation.profiledKd, Constants.simulation.profiledKi, Constants.simulation.profiledKd,
        new Constraints(Units.degreesToRadians(360),
        Units.degreesToRadians(180))
      ));
      ControllerConfigurator.configureControllerSim(this);
    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!
      ControllerConfigurator.configureControllerTest(this);
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void periodic() {
    limelightInstance.createVisionMeasurement(this);
  }
}