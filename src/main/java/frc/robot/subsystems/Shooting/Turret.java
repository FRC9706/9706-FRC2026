package frc.robot.subsystems.Shooting;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class Turret {
    // Create an instance for the Turret
    public static Turret mInstance = null;
    public static Turret getInstance() {
        if (mInstance == null) {
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
    private int[][] targetTags;

    // Initalize turret variables
    private double turretAngleDeg;   // Absolute turret angle

    // Face tracking variables
    private int[] currentFace = null;   // {tag1, tag2} of current alliance face
    private double bestFace = 0;         // For future scoring logic
    private double faceCenterTX = 0;     // PID target (face center)

    public static enum state {
        Idle,
        TagFinding,
        Tracking,
    }

    private state currentState = state.Idle;

    public state getState() {
        return currentState;
    }

    public void setState(state newState) {
        currentState = newState;
    }

    public Turret() {
        // Create the motor objects
        firingMotor = new TalonFX(Constants.Turret.firingMotor);
        rotationMotor = new TalonFX(Constants.Turret.rotationMotor);
        pitchMotor = new TalonFX(Constants.Turret.pitchMotor);

        // Create the CANcoder objects
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

    /* ==============================
       TAG FINDING (ROAM)
       ============================== */

    public void targetRoaming() {
        // Check if the variable actually has things in it, if not print out an error
        if (targetTags == null || targetTags.length == 0) {
            stopRotationMotor();  // No alliance = don't move
            System.out.println("Roaming failed, no april tag ID list! Check Alliance logic");
            return;
        }

        // If no target visible, keep roaming
        if (!tv) {
            rotationMotor.set(Constants.Turret.roamSpeed);
            return;
        }

        // If we see something, check if it's our alliance tag
        for (int[] face : targetTags) {
            if ((int) tagID == face[0] || (int) tagID == face[1]) {
                currentFace = face;
                setState(state.Tracking);
                stopRotationMotor();
                return;
            }
        }

        // Wrong tag  -> keep roaming
        rotationMotor.set(Constants.Turret.roamSpeed);
    }

    /* ==============================
       TARGET TRACKING
       ============================== */

    public void targetTracking() {
        // No targets -> back to TagFinding
        if (!tv || currentFace == null) {
            setState(state.TagFinding);
            return;
        }

        // Validate the currently visible tag still belongs to our face
        if ((int) tagID != currentFace[0] && (int) tagID != currentFace[1]) {
            setState(state.TagFinding);
            return;
        }

        // For now, use Limelight best target tx directly
        faceCenterTX = tx;

        // Deadband to prevent motor overshoot
        if (Math.abs(faceCenterTX) < Constants.Turret.txDeadbandDeg) {
            stopRotationMotor();
            return;
        }

        // Simple proportional control
        double rotationPower = faceCenterTX * Constants.Turret.kpRotation;

        // Clamp output power
        rotationPower = Math.copySign(
            Math.min(Math.abs(rotationPower), Constants.Turret.maxRotPower),
            rotationPower
        );

        rotationMotor.setControl(new DutyCycleOut(rotationPower));
    }

    public void getDistance() {
        // For future distance calculations
    }

    public double getTargetRotationAngle() {
        // Gets the angular error of the turret to the tag
        return faceCenterTX;
    }

    public void periodic() {
        // Asign limelight variables periodically to update continously
        tx = LimelightHelpers.getTX("turretLimelight");
        ty = LimelightHelpers.getTY("turretLimelight");
        tv = LimelightHelpers.getTV("turretLimelight");
        tagID = LimelightHelpers.getFiducialID("turretLimelight");

        // Asign turret rotational values for calculations
        turretAngleDeg = rotationCC.getAbsolutePosition().getValueAsDouble() * 360.0;

        // Get target tag IDs
        targetTags = Constants.Turret.Limelight.Tags.getAprilTags();

        // STATE LOGIC
        switch (currentState) {
            case Idle:
                stopFiringMotor();
                stopRotationMotor();
                stopTiltMotor();
                break;

            case TagFinding:
                targetRoaming();
                break;

            case Tracking:
                targetTracking();
                break;
        }
    }
}
