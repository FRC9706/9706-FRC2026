package frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;
import frc.robot.util.Networking.portForwardUtils;
import frc.robot.util.Tuning.LiveTuner;

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

    public void portFowardLL(String LL) {
        if (LL == "FR") {
        portForwardUtils.forwardPortRange(VisionConstants.limelightFrontRightIP, 
         5800, 5805, 5800, 5805);
        } else if (LL == "FL") {
        portForwardUtils.forwardPortRange(VisionConstants.limelightFrontLeftIP, 
         5800, 5805, 5800, 5805); 
        }

        if (LL == "BR") {
        portForwardUtils.forwardPortRange(VisionConstants.limelightBackRightIP, 
         5800, 5805, 5800, 5805); 
        } else if (LL == "BL") {
        portForwardUtils.forwardPortRange(VisionConstants.limelightBackLeftIP, 
         5800, 5805, 5800, 5805); 
        }
    }

    public void removeFowardedLL() {
        portForwardUtils.removeForwardedPorts(5800, 5805); 
    }

    public void createPortFowardSlider() {
            LiveTuner.choice(
            "PortForwarder/LL", 
            0, 
            new String[]{"FrontRight", "FrontLeft", "BackRight", "BackLeft"}, 
            LLindex -> {
                switch(LLindex) {
                    case 0:
                        removeFowardedLL();
                        portFowardLL("FR");
                        break;
                    case 1:
                        removeFowardedLL();
                        portFowardLL("FL");
                        break;
                    case 2:
                        removeFowardedLL();
                        portFowardLL("BR");
                        break;
                    case 3:
                        removeFowardedLL();
                        portFowardLL("BL");
                        break;
                }
            }
        );
    }
}
