package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class Turret {
    // Create an instance for the Turret
    public static Turret mInstance = null;
    public static Turret getInstance() {
       if(mInstance == null){
           mInstance = new Turret();
       }
       return mInstance;
   }

    // Initialize the motors for the turret
    private TalonFX firingMotor;
    private TalonFX rotationMotor;
    private TalonFX tiltMotor;

    // Initalize limelight variables
    private double tx;
    private double ty;
    private boolean tv;

    public static enum state {
        Idle,
        Roaming,
        Tracking,
        FcTracking
    }

    public void turret() {
        firingMotor = new TalonFX(Constants.Turret.firingMotor);
        rotationMotor = new TalonFX(Constants.Turret.rotationMotor);
        tiltMotor = new TalonFX(Constants.Turret.tiltMotor);

    }

    public void stopFiringMotor() {
        firingMotor.stopMotor();
    }

    public void stopRotationMotor() {
        rotationMotor.stopMotor();
    }

    public void stopTiltMotor() {
        tiltMotor.stopMotor();
    }

    public void getDistance() {
        
    }

    public void getTargetRotationAngle() {

    }

    public void periodic() {
        // Asign limelight variables in periodic to update continously
        tx = LimelightHelpers.getTX("limelight");
        ty = LimelightHelpers.getTY("limelight");
        tv = LimelightHelpers.getTV("limelight");
    }
}
