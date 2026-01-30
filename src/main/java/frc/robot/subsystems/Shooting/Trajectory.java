package frc.robot.subsystems.Shooting;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Trajectory {
    // Initializse Interpolataion Tree Maps
    private InterpolatingDoubleTreeMap turPitch;
    private InterpolatingDoubleTreeMap turFly;

    // Intialize Pose variables
    private final Supplier<Pose2d> poseSupplier;

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

    public Pose2d getPose2d() {
        return poseSupplier.get();
    }

    public Trajectory(Supplier<Pose2d> poseSupplier) {
        // map the pose supplier to the passed in pose supplier
        this.poseSupplier = poseSupplier;

        // load the tree map data
        loadTreeMaps();
    }
}
