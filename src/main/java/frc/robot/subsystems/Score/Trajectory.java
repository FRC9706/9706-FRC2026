package frc.robot.subsystems.Score;

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

    
    /**
     * Calculates turret position based on two encoder readings.
     * Logic identical to getTurretPosition() (minus print statements).
     *
     * @param encoderPos1 The first encoder position.
     * @param encoderPos2 The second encoder position.
     * @return The calculated turret position as a double.
     */
    public double calculateAndrewPos(double encoderPos1, double encoderPos2) {
        double r1 = 6, r2 = 7;
        double e1 = encoderPos1 % 1.0;
        double e2 = encoderPos2 % 1.0;

        int counter = 0;

        for (int i = 0; i < r1; i++) {
            double p1 = (e1 + i) / r1;
            double expected_e2 = (p1 * r2) % 1.0;
            counter++;

            if (Math.abs(expected_e2 - e2) < 0.01) {
                return p1 % 1.0;
            }
        }

        throw new RuntimeException("No match found after " + counter + " cycles");
    }
}
