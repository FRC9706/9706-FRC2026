// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.doomAndDespair.boomBox;
import frc.robot.util.Controller.ControllerConfigurator;
import frc.robot.util.Pathplanner.Preloader;
import lib.Networking.DynamicInputs;
import lib.Networking.DynamicInputs.DynamicChoice;
import lib.Swerve.SwerveConfigurator;
import lombok.Getter;
import frc.robot.subsystems.Floor.FloorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Define our controller on port 0
  @Getter private final CommandXboxController mDriverController = new CommandXboxController(Constants.Controller.kDriverControllerPort);

  // Create a drivebase
  @Getter private final SwerveSubsystem drivebase = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  // ----------------------------------------
  // Init instance based subsystems
  // ----------------------------------------

  // Init Controller configurator
  public final ControllerConfigurator controllerConfig = new ControllerConfigurator();

  // Init limelight subsystem
  @Getter private final VisionSubsystem limelight = new VisionSubsystem(drivebase.getSwerveDrive());

  // Init the Score subsystems
  //@Getter private final TurretSubsystem turret = new TurretSubsystem();
  //@Getter private final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  //@Getter private final HoodSubsystem hood = new HoodSubsystem();

  // Init spindexer subsystem
  // @Getter private final SpindexerSubsystem spindexer = new SpindexerSubsystem();

  // Init floor subsystem
  @Getter private final FloorSubsystem floor = new FloorSubsystem();

  // Init intake subsystem
  //@Getter private final IntakeSubsystem intake = new IntakeSubsystem();

  // Init superstructure
  //@Getter private final SuperStructure superStructure = new SuperStructure(intake, floor);

  // init doom and despair - boomBox subsystem
  @Getter private final boomBox boomBox = new boomBox(this);

  // ----------------------------------------
  // Initalize Variables
  // ----------------------------------------
  private DynamicChoice autoChoice;

  private String[] autosList;

  // ----------------------------------------
  // Get Variables
  // ----------------------------------------

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

    // Create a configurable slider for scale rotations (basically speed)
    DynamicInputs.DynamicNum(
      "Swerve/Controller Scales/Scale Translation",
     Constants.Controller.scaleTranslation,
      value -> {
        streams.configureAll(stream -> {
          stream.scaleTranslation(value);
        });
      }
    );

    DynamicInputs.DynamicNum(
      "Swerve/Controller Scales/Scale Rotation",
     Constants.Controller.scaleRotation,
      value -> {
        streams.configureAll(stream -> {
          stream.scaleRotation(value);
        });
      }
    );

    // Configure the trigger bindings
    setupBindings();

    // Preload any trajectories for Path Planner
    Preloader.preloadsAutos();

    // List of autos for auto choice
    autosList = new String[] {
      "No Auto",
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
            System.out.println("[Auto Choice]: choice updated! Index: " + 0 + " (No Auto)");
          break;
          case 1:
            System.out.println("[Auto Choice]: choice updated! Index: " + 1);
            Preloader.registerNamedCmds(autosList[autoIndex], this);
          break;
          case 2:
            System.out.println("[Auto Choice]: choice updated! Index: " + 2);
            Preloader.registerNamedCmds(autosList[autoIndex], this);
          break;
          case 3:
            System.out.println("[Auto Choice]: choice updated! Index: " + 3);
            Preloader.registerNamedCmds(autosList[autoIndex], this);
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

    // Set the driving method depending on mode (Real or Simulation)
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      controllerConfig.configureControllerSim(this);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      controllerConfig.configureControllerTL(this);
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
    if (getAutoChoice() == "No Auto") {
      System.out.println("[Auto Choice]: No auto selected!");
      return null;
    } else {
      System.out.println("Requesting autonomous command: " + getAutoChoice());
      return Preloader.getpath(getAutoChoice());
    }
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}