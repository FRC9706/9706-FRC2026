package frc.robot.subsystems.Shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Trajectory {
    private InterpolatingDoubleTreeMap turPitch;
    private InterpolatingDoubleTreeMap turFly;

    public void loadTreeMaps() {
        // Intialize the interpolating tree maps
        turPitch = new InterpolatingDoubleTreeMap();
        turFly = new InterpolatingDoubleTreeMap();
            // Add values to the tree maps
            turPitch.put(null, null);
            turPitch.put(null, null);
            turPitch.put(null, null);
            turPitch.put(null, null);

            turFly.put(null, null);
            turFly.put(null, null);
            turFly.put(null, null);
            turFly.put(null, null);
    }

    public Trajectory() {
        // load the tree map data
        loadTreeMaps();
    }
}
