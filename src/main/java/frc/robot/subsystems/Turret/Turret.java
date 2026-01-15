package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Turret {
    // Create an instance for the Turret
    public static Turret mInstance = null;
    public static Turret getInstance() {
       if(mInstance==null){
           mInstance = new Turret();
       }
       return mInstance;
   }

    // Initialize the motors for the turret
    private TalonFX firingMotor = new TalonFX(Constants.Turret.firingMotor);
    private TalonFX turnMotor = new TalonFX(Constants.Turret.turnMotor);
    private TalonFX angleMotor = new TalonFX(Constants.Turret.angleMotor);

    public static enum state {
        Idle,
        Roaming,
        Tracking,
        FcTracking
    }

    public static void turret() {
        
    }


    public static void periodic() {

    }
}
