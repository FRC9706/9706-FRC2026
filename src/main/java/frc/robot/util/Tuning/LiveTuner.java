package frc.robot.util.Tuning;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class LiveTuner {


    public static double[] arrayTuner(double[] pid) {
        double[] npid = new double[3];

        ShuffleboardTab tTab = Shuffleboard.getTab("Turret");

        GenericEntry kP = tTab
                    .add("kP", pid[0])
                    .withWidget(BuiltInWidgets.kTextView) // specify the widget here
                    .getEntry();
        
        GenericEntry kI = tTab
                    .add("kI", pid[1])
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
        
        GenericEntry kD = tTab
                    .add("KD", pid[2])
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();

        npid[0] = kP.getDouble(pid[0]);
        npid[1] = kI.getDouble(pid[1]);
        npid[2] = kD.getDouble(pid[2]);

        return npid;
    }
}
