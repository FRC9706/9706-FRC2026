package frc.robot.subsystems.Score.TurretModules;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public Pose2d getHubPos() {
        // PLACE HOLDER
        return new Pose2d(0, 0, new Rotation2d(0));
    }

    public double getTurretOffsetAng() {
        Pose2d hub = getHubPos();
        Pose2d robot = getPose2d();

        double relHubPosX = hub.getX() - robot.getX();
        double relHubPosY = hub.getY() - robot.getY();

        // Angle from robot to hub in FIELD coordinates (radians)
        double fieldAngle = Math.atan2(relHubPosY, relHubPosX);
        // Robot heading in radians
        double robotHeading = robot.getRotation().getRadians();

        // Convert to ROBOT-relative angle
        double turretOffset = fieldAngle - robotHeading;

        return turretOffset;
    }

    
    /**
     * Calculates turret position based on two encoder readings.
     * Logic identical to getTurretPosition() (minus print statements).
     *
     * @param motorPosRot The motor's position in rotations.
     * @param extEncoderPosRot An external encoder's position in rotations.
     * @return The calculated turret position as a double.
     */
    public double calculateAndrewPos(double motorPosRot, double extEncoderPosRot) {
        double motorRatio = 143/13;
        double extEncoderRatio = 143/11;

        double[] pos1 = new double[(int) motorRatio];
        for (int i = 0; i < motorRatio; i++) {
            pos1[i] = (motorPosRot + i) * extEncoderPosRot/motorRatio;
        }

        double[] pos2 = new double[(int) extEncoderRatio];
        for (int i = 0; i < extEncoderRatio; i++) {
            pos2[i] = (extEncoderPosRot + i) * motorPosRot/extEncoderRatio;
        }

        int counter = 0;

        for (double p1 : pos1) {
            for (double p2 : pos2) {
                counter++;
                if (Math.abs(p1 - p2) < 0.001) {
                    return p1 % 1.0;
                }
            }
        }

        return 404;
    }
}
