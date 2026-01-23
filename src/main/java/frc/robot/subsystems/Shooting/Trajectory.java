package frc.robot.subsystems.Shooting;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;

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
