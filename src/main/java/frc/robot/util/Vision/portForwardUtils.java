package frc.robot.util.Vision;
import edu.wpi.first.net.PortForwarder;
import frc.robot.subsystems.Vision.VisionConstants;


public class portForwardUtils {
    portForwardUtils(){} // This is a utility class; don't load constructor
    /**
     * Forwards all ports in [startPort, endPort] from the roboRIO
     * to the given remote host using the same remote port number.
     */
    public static void forwardPortRange(
        String remoteHost, 
        int localStartPort, 
        int localEndPort,
        int remoteStartPort, 
        int remoteEndPort
    ) {
        for (int i = 0; i <= localEndPort - localStartPort; i++) {
            int localPort = localStartPort + i;
            int remotePort = remoteStartPort + i;
            PortForwarder.add(localPort, remoteHost, remotePort);
        }
    }


    public static void portFoward() {
        forwardPortRange(VisionConstants.limelightTurretIP, 
         5800, 5805, 5800, 5805);
    
        // forwardPortRange(Constants.limelights.limelightFrontIP, 
        //    5800, 5809, 5800, 5809);
    }
}
