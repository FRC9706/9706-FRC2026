package frc.robot.subsystems.Vision;

// import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
// import edu.wpi.first.math.geometry.Pose2d;
// import frc.robot.subsystems.Vision.LimelightHelpers;

public class Limelight {
   public static Limelight mInstance = null;
   public static Limelight getInstance() {
       if(mInstance==null){
           mInstance = new Limelight();
       }
       return mInstance;
   }

   public void createVisionMeasurements(RobotContainer container) {
    // The limelight for the turret
      container.getDrivebase().createVisionMeasurement("turretLimelight");
    // The front & back limelights
      container.getDrivebase().createVisionMeasurement("frontLimelight");
      container.getDrivebase().createVisionMeasurement("backLimelight");
    // The side limelights
      container.getDrivebase().createVisionMeasurement("rightLimelight");
      container.getDrivebase().createVisionMeasurement("leftLimelight");
   }

}
