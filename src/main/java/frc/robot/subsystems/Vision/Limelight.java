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

   public void createVisionMeasurements(RobotContainer container, double timeStamp) {
    // Front
    container.getDrivebase().createVisionMeasurement("limelight-frontri", timeStamp);
    container.getDrivebase().createVisionMeasurement("limelight-frontle", timeStamp);
    // Back
    container.getDrivebase().createVisionMeasurement("limelight-backri", timeStamp);
    container.getDrivebase().createVisionMeasurement("limelight-backle", timeStamp);
   }
}
