package frc.robot.subsystems.Score.Turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Field.FieldConstants;
import lib.Geometry.Translation2d;
import lombok.Getter;

public class TurretSubsystem extends SubsystemBase {

  @Getter private TalonFX turretMotor;
  private MotionMagicVoltage request = new MotionMagicVoltage(0);

  public TurretSubsystem() {

    turretMotor = new TalonFX(TurretConstants.rotationMotor);
    turretMotor.getConfigurator().apply(TurretConstants.config);

    turretMotor.setPosition(0);

  }

  public void setTurretPosition(double rotations) {

    double minAngle = 10;
    double maxAngle = 410;

    double newSetpoint = Units.rotationsToDegrees(rotations);

    // Wrap the input angle to [0, 360) degrees
    double clampedInput = newSetpoint % 360;
    if (clampedInput < 0) {
      clampedInput = clampedInput + 360;
    }

    if (Math.abs(clampedInput - (Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble()) - minAngle)) > 180) {
      /*if the desired position is that far away from the current position, 
      then we want to check if we can go the other way! */
      if (clampedInput + 360 < maxAngle) {
        clampedInput += 360;
      }
    }

    request = new MotionMagicVoltage(Units.degreesToRotations(clampedInput));
  }

  public void targetHub(Pose2d robotPose) {
        Translation2d hub = FieldConstants.Hub.hubCenterPoint2d.mirrorAboutX(FieldConstants.LinesVertical.center);

        double relHubPosX = hub.x() - robotPose.getX();
        double relHubPosY = hub.y() - robotPose.getY();

        Rotation2d fieldAngle =  Rotation2d.fromRadians(Math.atan2(relHubPosY, relHubPosX));
      
        double turretHeading = fieldAngle.minus(robotPose.getRotation()).getRotations();

        setTurretPosition(turretHeading);
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Turret/Setpoint", Units.rotationsToDegrees(request.Position));
    Logger.recordOutput("Turret/Position", Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble()));
    Logger.recordOutput("Turret/Velocity", Units.rotationsToDegrees(turretMotor.getVelocity().getValueAsDouble()));
    Logger.recordOutput("Turret/AppliedVoltage", turretMotor.getMotorVoltage().getValueAsDouble());

    turretMotor.setControl(request);
  }
}
