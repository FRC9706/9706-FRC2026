package frc.robot.util.Vision;
import edu.wpi.first.net.PortForwarder;
import frc.robot.Constants;

public class portForwardUtils {
    portForwardUtils(){} // This is a utility class; don't load constructor
    /**
     * Forwards all ports in [startPort, endPort] from the roboRIO
     * to the given remote host using the same remote port number.
     */
    public static void forwardPortRange(String remoteHost, int startPort, int endPort) {
        for (int port = startPort; port <= endPort; port++) {
            PortForwarder.add(port, remoteHost, port);
        }
    }

    public static void portFoward() {
        forwardPortRange(Constants.limelights.limelightTurretIP, 5800, 5809);
    }
}
