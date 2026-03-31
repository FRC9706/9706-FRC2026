package frc.robot.subsystems.Score.Hood;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class HoodSubsystem extends SubsystemBase {

  @Getter private TalonFX hoodMotor;
  private MotionMagicVoltage request = new MotionMagicVoltage(0);
  private InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  public HoodSubsystem() {

    hoodMotor = new TalonFX(HoodConstants.motorID);
    hoodMotor.getConfigurator().apply(HoodConstants.config);
    hoodMotor.setPosition(Units.degreesToRotations(HoodConstants.minAngle));

    hoodMap.put(1d, Units.degreesToRotations(HoodConstants.minAngle));
    hoodMap.put(2d, Units.degreesToRotations(HoodConstants.maxAngle));

  }

  public void setHoodPosition(double rotations) {
    request = new MotionMagicVoltage(rotations);
  }

  public void targetHub(double distance) {
    setHoodPosition(hoodMap.get(distance));
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Hood/Setpoint", Units.rotationsToDegrees(request.Position));
    Logger.recordOutput("Hood/Position", Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble()));
    Logger.recordOutput("Hood/Velocity", Units.rotationsToDegrees(hoodMotor.getVelocity().getValueAsDouble()));
    Logger.recordOutput("Hood/AppliedVoltage", hoodMotor.getMotorVoltage().getValueAsDouble());

    hoodMotor.setControl(request);
  }
}
