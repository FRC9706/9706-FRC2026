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
import frc.robot.util.Networking.DynamicInputs;
import frc.robot.util.Networking.DynamicInputs.DynamicChoice;
import frc.robot.util.Pathplanner.Preloader;
import frc.robot.util.Swerve.SwerveConfigurator;
import lombok.Getter;
import frc.robot.subsystems.Score.TurretModules.TurretMath;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.Intake;
// import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Score.TurretBeta;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


import java.io.File;

import swervelib.SwerveInputStream;
// import swervelib.parser.PIDFConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Define our controller on port 0
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Controller.kDriverControllerPort);
  // Create an instance for our Controller configurator
  public final ControllerConfigurator controllerConfiguratorInstance = ControllerConfigurator.getInstance();

  // Create a drivebase
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  // Init limelight subsystem
  @Getter private final Limelight limelightInst = Limelight.getInstance();

  // Init the Turret subsysem
  @Getter private final TurretBeta turretBetaInst = TurretBeta.getInstance(
    getDrivebase(), getTurretMathInst()
  );
  @Getter private final TurretMath turretMathInst = TurretMath.getInstance(drivebase.getPose());

  // Init spindexer subsystem
  @Getter private final Spindexer spindexerInst = Spindexer.getInstance();

  // Init hopper subsystem
  @Getter private final Hopper hopperInst = Hopper.getInstance();
  
  // init intake subsystem
  @Getter private final Intake intakeInst = Intake.getInstance();

  // ----------------------------------------
  // Initalize Variables
  // ----------------------------------------
  private DynamicChoice autoChoice;

  private String[] autosList;

  // ----------------------------------------
  // Get Variables
  // ----------------------------------------

  // Get controller
  public CommandXboxController getDriverController() {
    return m_driverController;
  }

  // Get drivebase
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  // Get the name of the chosen auto choice
  public String getAutoChoice() {
    return autoChoice.getSelected();
  }


  // Enums for the swerve input streams
  public enum SwerveStreamType {
    AngularVelocity,
    DirectAngle,
    RobotOreinted,
    AngularVelocityKeyboard,
    DirectAngleKeyboard
  }

  // Method to get swerve input streams
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
            // Handle error case; return null
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

    // Preload any trajectories for Path Planner
    Preloader.preloadsAutos();

    // Load Port Fowarder
    limelightInst.createPortFowardSlider();

    // List of autos for auto choice
    autosList = new String[] {
      "rightAuto",
      "centerAuto",
      "leftAuto"
    };
    // Setup an auto coser
    autoChoice = DynamicInputs.choice(
      "Auto/chosenPath", 
      0, 
      autosList,
      autoIndex -> {
        switch (autoIndex) {
          case 0:
            System.out.println("Auto choice updated! Index: " + 0);
            Preloader.registerNamedCmds(autosList[autoIndex]);
          break;
          case 1:
            System.out.println("Auto choice updated! Index: " + 1);
            Preloader.registerNamedCmds(autosList[autoIndex]);
          break;
          case 2:
            System.out.println("Auto choice updated! Index: " + 2);
            Preloader.registerNamedCmds(autosList[autoIndex]);
          break;
        }
      }
    );

  // After swerveDrive is fully created, attach live tuning:
  // var modules = drivebase.getSwerveDrive().getModules();

  // tune drive PID on all modules together
  // DynamicInputs.pid(
  //     "Swerve/DrivePID",
  //     SwerveConstants.Swerve.translationP,
  //     SwerveConstants.Swerve.translationI,
  //     SwerveConstants.Swerve.translationD,
  //     (p, i, d) -> {
  //       for (var module : modules) {
  //         System.out.println("Live drive PID: " + p + ", " + i + ", " + d);
  //         PIDFConfig config = new PIDFConfig(p, i, d, 0);
  //         module.setDrivePIDF(config);
  //       }
  //     });

  // DynamicInputs.pid(
  //     "Swerve/AnglePID",
  //     SwerveConstants.Swerve.rotationP,
  //     SwerveConstants.Swerve.rotationI,
  //     SwerveConstants.Swerve.rotationD,
  //     (p, i, d) -> {
  //       for (var module : modules) {
  //         System.out.println("Live Angle PID: " + p + ", " + i + ", " + d);
  //         PIDFConfig config = new PIDFConfig(p, i, d, 0);
  //         module.setAnglePIDF(config);
  //       }
  //     });
  }

  /**
   * Use this method to define your trigger -> command mappings. Triggers can be created via the
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
      controllerConfiguratorInstance.configureControllerTL(this);
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
    System.out.println("Request autonmous command: " + getAutoChoice());
    return Preloader.getpath(getAutoChoice());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}