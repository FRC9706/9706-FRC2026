package frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;
import frc.robot.util.Networking.DynamicInputs;
import frc.robot.util.Networking.PortForwardUtils;

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
        PortForwardUtils.forwardPortRange(VisionConstants.limelightFrontRightIP, 
         5800, 5805, 5800, 5805);
        } else if (LL == "FL") {
        PortForwardUtils.forwardPortRange(VisionConstants.limelightFrontLeftIP, 
         5800, 5805, 5800, 5805); 
        }

        if (LL == "BR") {
        PortForwardUtils.forwardPortRange(VisionConstants.limelightBackRightIP, 
         5800, 5805, 5800, 5805); 
        } else if (LL == "BL") {
        PortForwardUtils.forwardPortRange(VisionConstants.limelightBackLeftIP, 
         5800, 5805, 5800, 5805); 
        }
    }

    public void removeFowardedLL() {
        PortForwardUtils.removeForwardedPorts(5800, 5805); 
    }

    public void createPortFowardSlider() {
            DynamicInputs.choice(
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
