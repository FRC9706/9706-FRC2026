package frc.robot.util.Networking;
import edu.wpi.first.net.PortForwarder;


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
}
