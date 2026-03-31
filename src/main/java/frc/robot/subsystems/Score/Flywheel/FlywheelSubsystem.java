package frc.robot.subsystems.Score.Flywheel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

  private TalonFX[] motors;
  private VelocityVoltage request = new VelocityVoltage(0);

  private InterpolatingDoubleTreeMap flyMap = new InterpolatingDoubleTreeMap();

  public FlywheelSubsystem() {
    motors = new TalonFX[] {
      new TalonFX(FlywheelConstants.motorIDs[0]),
      new TalonFX(FlywheelConstants.motorIDs[1])
    };

    for (TalonFX motor : motors) {
      motor.getConfigurator().apply(FlywheelConstants.config);
    }

    flyMap.put(1d, 1000d);
    flyMap.put(2d, 2000d);

  }

  public void setFlywheelVelocity(double rpm) {
    request = new VelocityVoltage(rpm/60);
  }

  public void targetHub(double distance) {
    setFlywheelVelocity(flyMap.get(distance));
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Flywheel/Velocity", Units.rotationsToDegrees(motors[0].getVelocity().getValueAsDouble()));
    Logger.recordOutput("Flywheel/AppliedVoltage", motors[0].getMotorVoltage().getValueAsDouble());

    motors[1].setControl(new Follower(FlywheelConstants.motorIDs[0], MotorAlignmentValue.Opposed));
    motors[0].setControl(request);
  }
}
