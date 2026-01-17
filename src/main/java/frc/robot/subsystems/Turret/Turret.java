package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;

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
    private TalonFX pitchMotor;

    // Initalize limelight variables
    private double tx;
    private double ty;
    private boolean tv;

    // Initalize turret variables
    private double rotationPos;

    public static enum state {
        Idle,
        Roaming,
        Tracking,
        FcTracking
    }

    public void turret() {
        firingMotor = new TalonFX(Constants.Turret.firingMotor);
        rotationMotor = new TalonFX(Constants.Turret.rotationMotor);
        pitchMotor = new TalonFX(Constants.Turret.pitchMotor);

    }

    public void stopFiringMotor() {
        firingMotor.stopMotor();
    }

    public void stopRotationMotor() {
        rotationMotor.stopMotor();
    }

    public void stopTiltMotor() {
        pitchMotor.stopMotor();
    }

    public double getTargetRotationAngle() {
        // Gets the angular error of the turret to the tag
        double e = rotationPos - tx;
        return e;
    }

    public void getDistance() {
        
    }

    public void periodic() {
        // Asign limelight variables periodically to update continously
        tx = LimelightHelpers.getTX("limelight");
        ty = LimelightHelpers.getTY("limelight");
        tv = LimelightHelpers.getTV("limelight");

        // Asign turret rotational values for calculations
        rotationPos = rotationMotor.getPosition().getValueAsDouble();
    }
}
