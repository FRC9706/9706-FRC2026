package frc.robot.subsystems.Score.TurretModules;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.Field.FieldConstants;
import frc.robot.subsystems.Score.TurretConstants;
import lib.Alliance.AllianceUtils;
import lib.Geometry.Translation2d;
import lombok.Getter;

public class TurretMath {
    // Singleton instance
    private static TurretMath mInstance = null;
    public static synchronized TurretMath getInstance(Pose2d robotPose) {
        if (mInstance == null) {
            mInstance = new TurretMath(robotPose);
        }
        return mInstance;
    }

    // Intialize variables
    @Getter private Pose2d robotPose;
    @Getter private Translation2d robotPoseTranslation;

    @Getter private InterpolatingDoubleTreeMap treeMapTurretRPM;
    @Getter private InterpolatingDoubleTreeMap treeMapHoodPos;


    public TurretMath(Pose2d robotPoseInput) {
        // map the pose supplier to the passed in pose supplier
        robotPose = robotPoseInput;
        robotPoseTranslation = new Translation2d(robotPose.getX(), robotPose.getY());

        // Initalize interpolating tree maps with points
        // treeMapTurretRPM = new InterpolatingDoubleTreeMap();
        //     treeMapTurretRPM.put(null, null);

        // treeMapHoodPos = new InterpolatingDoubleTreeMap();
        //     treeMapHoodPos.put(null, null);
    }

    public Translation2d getHubTranslation() {
        switch (AllianceUtils.getCurrentAlliance()) {
            case Blue:
                return FieldConstants.Hub.hubCenterPoint2d;
            case Red:
                return FieldConstants.Hub.hubCenterPoint2d.mirrorAboutX(FieldConstants.LinesVertical.center);
            default:
                return FieldConstants.Hub.hubCenterPoint2d;
        }
    }

    public double getRobotDistHub() {
        return robotPoseTranslation.distance(getHubTranslation());
    }
    
    public double getTurretOffsetToTranslation(Translation2d target) {

        double relTargetPosX = robotPose.getX() - target.x();
        double relTargetPosY = robotPose.getY() - target.y();

        // Angle from robot to hub in FIELD coordinates (radians)
        double fieldAngle =  Math.atan2(relTargetPosY, relTargetPosX);

        // Robot heading in radians
        double robotHeading = robotPose.getRotation().getRadians();

        // Convert to ROBOT-relative angle
        double turretOffset = fieldAngle - robotHeading;

        return turretOffset / (2 * Math.PI);
    }
    
    public double getTurretOffsetRot() {
        Translation2d hub = getHubTranslation();

        double relHubPosX = hub.x() - robotPose.getX();
        double relHubPosY = hub.y() - robotPose.getY();

        Logger.recordOutput("Turret/Math/HubOffsetCalculations/relHubX", (relHubPosX));
        Logger.recordOutput("Turret/Math/HubOffsetCalculations/relHubY", (relHubPosY));
        
        // Angle from robot to hub in FIELD coordinates (radians)
        double fieldAngle = Math.atan2(relHubPosY, relHubPosX);
        
        // Robot heading in radians
        double robotHeading = robotPose.getRotation().getRadians();
        
        // Convert to ROBOT-relative angle
        double turretOffset = fieldAngle - robotHeading;
        Logger.recordOutput(
            "Turret/Math/HubOffsetCalculations/turretOffset", 
            (turretOffset / (2 * Math.PI))
        );
        
        return turretOffset / (2 * Math.PI);
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
        return (((x % 1) + 1) % 1);
    }

    public double newNewBRT(double encoder1Pos, double encoder2Pos) {
        encoder1Pos = wrap(encoder1Pos);
        encoder2Pos = wrap(encoder2Pos);

        double predictedPos = 404;
        double smallestEncoder2PosError = Double.POSITIVE_INFINITY;
        
        int encoder1Teeth = TurretConstants.rotMotorTeeth;
        int encoder2Teeth = TurretConstants.extEncoderTeeth;
        int centerGearTeeth = TurretConstants.centerGearTeeth;

        double encoder1ToCenterRatio = (double)encoder1Teeth/(double)centerGearTeeth;
        double encoder2ToCenterRatio = (double)encoder2Teeth/(double)centerGearTeeth;

        // generate list of possible positions of center gear where encoder1Pos = encoder1Pos
        double[] possiblePos1 = new double[encoder2Teeth];
        for (int i = 0; i < encoder2Teeth; i++) {
            possiblePos1[i] = (encoder1Pos * encoder1ToCenterRatio) + (encoder1ToCenterRatio * i);
        }
        Logger.recordOutput("Turret/Math/ART/Calculations/KrakenPossibilities", possiblePos1);

        // generate list of possible positions of encoder2Pos for possilbe center gear positions
        double[] possiblePos2 = new double[encoder1Teeth];
        for (int i = 0; i < encoder1Teeth; i++) {
            possiblePos2[i] = (encoder2Pos * encoder2ToCenterRatio) + (encoder2ToCenterRatio * i);
        }
        Logger.recordOutput("Turret/Math/ART/Calculations/ExtEncoderPossibilities", possiblePos2);

        //find possible encoder2Pos that produces the least error with the actual, then use the possiblePos of the center gear it is associated with
        for (int i = 0; i < possiblePos1.length; i++) {
            for (int j = 0; j < possiblePos2.length; j++) {
                // double error = Math.abs(possiblePos1[i] - possiblePos2[j]);
                // remember that cuz circle: 0.1 is closer to 0.9 than 0.6
                // if (error > (1 - error)) {
                // error = 1 - error;
                // }

                // 10 billion iq?
                double diff = Math.abs(possiblePos1[i] - possiblePos2[j]);
                double error = Math.min(diff, 1 - diff);
                
                if (error < smallestEncoder2PosError) {
                predictedPos = possiblePos1[i];
                smallestEncoder2PosError = error;
                }
           }
        }

        
        return predictedPos;
    }
}
