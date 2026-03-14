package frc.robot.util.Networking;
import edu.wpi.first.net.PortForwarder;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.util.Tuning.LiveTuner;


public class portForwardUtils {
    portForwardUtils() {} // Static utility; don't load constructor
    /**
     * Forwards all ports in [startPort, endPort] from the roboRIO
     * to the given remote host using the same remote port number.
     */
    public static void forwardPortRange(
        String remoteHost, 
        int localStartPort, 
        int localEndPort,
        int remoteStartPort, 
        int remoteEndPort) {
        for (int i = 0; i <= localEndPort - localStartPort; i++) {
            int localPort = localStartPort + i;
            int remotePort = remoteStartPort + i;
            PortForwarder.add(localPort, remoteHost, remotePort);
        }
    }

    public static void removeForwardedPorts(
        int remoteStartPort, 
        int remoteEndPort) {
        for (int i = 0; i <= remoteEndPort - remoteStartPort; i++) {
            int remotePort = remoteStartPort + i;
            PortForwarder.remove(remotePort);
        }
    }


    public static void portFowardLL(String LL) {
        if (LL == "FR") {
        forwardPortRange(VisionConstants.limelightFrontRightIP, 
         5800, 5805, 5800, 5805);
        } else if (LL == "FL") {
        forwardPortRange(VisionConstants.limelightFrontLeftIP, 
         5800, 5805, 5800, 5805); 
        }

        if (LL == "BR") {
        forwardPortRange(VisionConstants.limelightBackRightIP, 
         5800, 5805, 5800, 5805); 
        } else if (LL == "BL") {
        forwardPortRange(VisionConstants.limelightBackLeftIP, 
         5800, 5805, 5800, 5805); 
        }
    }

    public static void removeFowardedLL() {
        removeForwardedPorts(5800, 5805); 
    }

    public static void createPortFowardSlider() {
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
