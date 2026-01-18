package frc.robot.util.Alliance;

import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtils {
    public enum Alliance {
        Blue, Red, Unknown
    }
    
    public static Alliance getCurrentAlliance() {
        var wpilibAlliance = DriverStation.getAlliance();
        if (wpilibAlliance.isEmpty()) {
            return Alliance.Unknown;
        }
        return wpilibAlliance.get() == DriverStation.Alliance.Blue
            ? Alliance.Blue 
            : Alliance.Red;
    }
    
    public static boolean hasAlliance() {
        return getCurrentAlliance() != Alliance.Unknown;
    }
}
