package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Networking.DynamicInputs;
import lib.Networking.PortForwardUtils;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
    // init variables
    private final SwerveDrive mSwerveDrive;

    public VisionSubsystem(SwerveDrive swerveDriveInput) {
        mSwerveDrive = swerveDriveInput;

        // Create an auto-port forwardering slider
        createPortFowardSlider();
    }

   public void factorVisionMeasurements(double timeStamp) {
    Matrix<N3,N1> stdDevs = VecBuilder.fill(0.4,0.4, Units.degreesToRadians(30));

        for (String llName : VisionConstants.limelightNames) {
            /*
            This weird logic is being done due to the fact that .getBotPoseEstimate 
            is a weird function and it can return weird null values.

            Therefore, I had to use an Optional<> variable, specifically .ofNullable since
            it can return a null value. I want to clarify this is avoiding a massive failure
            and java execption and not just being negligent.
            */
           try {
                Pose2d llPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName).pose;
                 double tagCount = LimelightHelpers.getTargetCount(llName);

                if ((llPoseEstimate != null) && (tagCount >= 1)) {
                    mSwerveDrive.addVisionMeasurement(
                        llPoseEstimate, 
                        timeStamp, 
                        stdDevs
                    );
                }

                // log the limelight's pose
                Logger.recordOutput(
                    "Vision/PoseEstimates/" + llName, 
                    llPoseEstimate
                );

            } catch (NullPointerException e) {
                // log exeption instead of pose
                Logger.recordOutput(
                    "Vision/PoseEstimates/" + llName, 
                    "Attempted pose estimate read for " + llName + " failed due to: " + e
                );
            }
        }
   }

   @Override
   public void periodic() {
        // Factor vision measurements
        factorVisionMeasurements(
           Timer.getTimestamp()
        );
   }

    public void portFowardLL(String LL) {
        if (LL == "FR") {
        PortForwardUtils.forwardPortRange(VisionConstants.limelightFrontRightIP, 
         5800, 5805, 5800, 5805);
        } else if (LL == "FL") {
        PortForwardUtils.forwardPortRange(VisionConstants.limelightFrontLeftIP, 
         5800, 5805, 5800, 5805); 
        }

        if (LL == "BR") {
        PortForwardUtils.forwardPortRange(VisionConstants.limelightBackRightIP, 
         5800, 5805, 5800, 5805); 
        } else if (LL == "BL") {
        PortForwardUtils.forwardPortRange(VisionConstants.limelightBackLeftIP, 
         5800, 5805, 5800, 5805); 
        }
    }

    public void removeFowardedLL() {
        PortForwardUtils.removeForwardedPorts(5800, 5805); 
    }

    public void createPortFowardSlider() {
            DynamicInputs.choice(
            "PortForwarder/LL", 
            0, 
            new String[]{"FrontRight", "FrontLeft", "BackRight", "BackLeft"}, 
            LLindex -> {
                switch(LLindex) {
                    case 0:
                        removeFowardedLL();
                        portFowardLL("FR");
                        break;
                    case 1:
                        removeFowardedLL();
                        portFowardLL("FL");
                        break;
                    case 2:
                        removeFowardedLL();
                        portFowardLL("BR");
                        break;
                    case 3:
                        removeFowardedLL();
                        portFowardLL("BL");
                        break;
                }
            }
        );
    }
}
