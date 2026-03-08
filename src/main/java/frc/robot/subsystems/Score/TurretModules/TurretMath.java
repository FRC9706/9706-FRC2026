package frc.robot.subsystems.Score.TurretModules;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Score.TurretConstants;
import frc.robot.util.Tuning.LiveTuner;

public class TurretMath {
    // Intialize Pose variables
    private final Supplier<Pose2d> poseSupplier;

    // Temp tuning
    private final LiveTuner.TunableNumber toleranceART;

    public TurretMath(Supplier<Pose2d> poseSupplier) {
        // map the pose supplier to the passed in pose supplier
        this.poseSupplier = poseSupplier;

        // temp tuning
        toleranceART = LiveTuner.number("Turret/toleranceART", 0.02);
    }
    
    public Pose2d getPose2d() {
        return poseSupplier.get();
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
     * Find the best position within turret rotation limits.
     * Tries wrapping by +/- 2π to find the shortest path.
     * 
     * @param targetRot target position in rotations
     * @param currentRot current position in rotations
     * @param turretRotLim max rotation limit in rotations
     * @return best position within limits, or clamped position if no valid path
     */
    public double findFastestPos(
        double targetRot, double currentRot, double turretRotLim) {

        double bestAng = targetRot;
        double minLim = -turretRotLim;
        double maxLim = turretRotLim;
        double minDistance = Math.abs(targetRot - currentRot);

        // Try wrapping +/- 1 rotation to find shortest path
        for (int i = -1; i <= 1; i++) {
            double candidate = targetRot + i;  // wrapping by 1 rotation
            if (candidate >= minLim && candidate <= maxLim) {
                double distance = Math.abs(candidate - currentRot);
                if (distance < minDistance) {
                    bestAng = candidate;
                    minDistance = distance;
                }
            }
        }

        return MathUtil.clamp(bestAng, minLim, maxLim);
    }

    
    /**
     * Calculates turret position based on two encoder readings.
     * Logic identical to getTurretPosition() (minus print statements).
     *
     * @param motorPosRot The motor's position in rotations.
     * @param extEncoderPosRot An external encoder's position in rotations.
     * @return The calculated turret position as a double.
     */

    public double wrap(double x) {
        return (x % 1.0 + 1.0) % 1.0;
    }

    public double calculateAndrewPos(double motorPosRot, double extEncoderPosRot) {
        // input filtering
        // motorPosRot = wrap(motorPosRot);
        // extEncoderPosRot = wrap(extEncoderPosRot);

        double motorRatio = TurretConstants.rotMotorGearRatio;
        double extEncoderRatio = TurretConstants.extEncoderGearRatio;

        int motorTeeth = TurretConstants.rotMotorTeeth;
        int extTeeth = TurretConstants.extEncoderTeeth;

        double[] pos1 = new double[motorTeeth];
        for (int i = 0; i < motorTeeth; i++) {
            pos1[i] = wrap((motorPosRot + i) / motorRatio);
        }

        double[] pos2 = new double[extTeeth];
        for (int i = 0; i < extTeeth; i++) {
            pos2[i] = wrap((extEncoderPosRot + i) / extEncoderRatio);
        }

        for (double p1 : pos1) {
            for (double p2 : pos2) {

                if (Math.abs(p1 - p2) < (toleranceART.get())) {
                    return p1;
                }
            }
        }
        return 404;
    }
}
