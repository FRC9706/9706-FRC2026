package frc.robot.subsystems.Vision;

import edu.wpi.first.net.PortForwarder;
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
    // The limelight for the turret
      container.getDrivebase().createVisionMeasurement("limelight-turret", timeStamp);
    // The front & back limelights
    //   container.getDrivebase().createVisionMeasurement("frontLimelight", timeStamp);
    //   container.getDrivebase().createVisionMeasurement("backLimelight", timeStamp);
    // // The side limelights
    //   container.getDrivebase().createVisionMeasurement("rightLimelight", timeStamp);
    //   container.getDrivebase().createVisionMeasurement("leftLimelight", timeStamp);
   }
}
