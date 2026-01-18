package frc.robot.subsystems.Shooting;

import com.ctre.phoenix6.hardware.CANcoder;
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

    // Initialize Can Coders for the turret motors
    private CANcoder rotationCC;
    private CANcoder pitchCC;

    // Initalize limelight variables
    private double tx;
    private double ty;
    private boolean tv;
    private double tagID;
    private int[] targetTags;

    // Initalize turret variables
    private double rotationPos;

    public static enum state {
        Idle,
        TagFinding,
        Tracking,
    //    FcTracking
    }

    private state currentState = state.Idle;

    public void setState(state newState) {
        currentState = newState;
    }

    public Turret() {
        // Create the motor objects
        firingMotor = new TalonFX(Constants.Turret.firingMotor);
        rotationMotor = new TalonFX(Constants.Turret.rotationMotor);
        pitchMotor = new TalonFX(Constants.Turret.pitchMotor);

        // Create the can coder objects
        rotationCC = new CANcoder(Constants.Turret.rotationCanCoder);
        pitchCC = new CANcoder(Constants.Turret.pitchCanCoder);

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

    public void targetRoaming() {
        // Check if the variable actually has things in it, if not print out an error
        if (targetTags.length == 0) {
            stopRotationMotor();  // No alliance = don't move
            System.out.println("Roaming failed, no april tag ID list! Check Alliance logic");
            return;
        }

        // Check if we see a valid target
        if (!tv) {
            // No target detected -> keep roaming
            rotationMotor.set(Constants.Turret.roamSpeed);
            return;
        }

        // We see something, check if it's our alliance tag
        boolean isValidTag = false;
        for (int validId : targetTags) {
            if ((int)tagID == validId) {
                isValidTag = true;
                break;
            }
        }

        if (isValidTag) {
            // Found a tag, switching to tracking mode
            setState(state.Tracking);
            stopRotationMotor();
                } else {
            // Wrong tag or enemy tag -> keep roaming
            rotationMotor.set(Constants.Turret.roamSpeed);
        }
    }

    public double getTargetRotationAngle() {
        // Gets the angular error of the turret to the tag
        double e = rotationPos - tx; // WIP, prolly wrong
        return e;
    }

    public void getDistance() {
        
    }

    public void periodic() {
        // Asign limelight variables periodically to update continously
        tx = LimelightHelpers.getTX("turretLimelight");
        ty = LimelightHelpers.getTY("turretLimelight");
        tv = LimelightHelpers.getTV("turretLimelight");

        // Asign turret rotational values for calculations
        rotationPos = rotationMotor.getPosition().getValueAsDouble();

        // Get target tag IDs
        targetTags = Constants.Turret.Limelight.Tags.getAprilTags();

        // STATE LOGIC
        tagID = LimelightHelpers.getFiducialID("turretLimelight");

        switch (currentState) {
            // What should be done in the idle state?
            case Idle:
                stopFiringMotor();
                stopRotationMotor();
                stopTiltMotor();
            break;

            // What should be done in the tag finding state?
            case TagFinding:
                targetRoaming();
            break;

            // What should be done in the tracking state?
            case Tracking:
                stopFiringMotor();
            break;
        }
    }
}
