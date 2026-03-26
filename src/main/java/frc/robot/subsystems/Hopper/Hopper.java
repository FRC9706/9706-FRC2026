package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Networking.DynamicInputs;
import lombok.Getter;

public class Hopper extends SubsystemBase {
    // Singleton instance
    private static Hopper mInstance = null;
    public static synchronized Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    // State machine
    @Getter private hopperState currHopperState = hopperState.IDLE;

    public enum hopperState {
        IDLE,
        EXTENDED,
        RETRACTED,
        WIGGLE
    }

    public void setHopperState(hopperState state) {
        currHopperState = state;
    } 

    public void findState() {
        if (hasReachedPos(HopperConstants.retractedPos)) {
            setHopperState(hopperState.RETRACTED);
        } else if (hasReachedPos(HopperConstants.extendedPos)) {
            setHopperState(hopperState.EXTENDED);
        } else {
            setHopperState(hopperState.IDLE);
        }
    }
    

    // initalize hardware variables
    TalonFX[] hopperMotors;

    // initalize motion profile variable
    private MotionMagicVoltage m_request;

    public Hopper() {
        // intialize hopper motors
        hopperMotors = new TalonFX[] {
            new TalonFX(HopperConstants.hopperMotors[0]),
            new TalonFX(HopperConstants.hopperMotors[1])
        };

        // Configure hopper motors
        MotorConfigs.configureMotors(hopperMotors);

        // setup live tuner for the hopper motors
        DynamicInputs.pidMagicMotion("Hopper/HopperPID", 
            HopperConstants.kPID[0], 
            HopperConstants.kPID[1], 
            HopperConstants.kPID[2], 
            HopperConstants.kPID[3],
            HopperConstants.kPID[4],
            HopperConstants.kPID[5],
            HopperConstants.kPID[6],
            HopperConstants.kPID[7],
            HopperConstants.kPID[8],
            (p, i, d, 
                s, y, b, a, u, 
            ok) -> 
                MotorConfigs.configurePID(
                    hopperMotors, p, i, d, 
                        s, y, b, a, u, 
                    ok)
        );
    }

    public double getHopperPos() {
        return hopperMotors[0].getPosition().getValueAsDouble();
    }

    public void stopHopperMotors() {
        hopperMotors[0].stopMotor();
    }

    public void moveHopperToPos(double pos) {
        m_request = new MotionMagicVoltage(0).withPosition(pos);
        hopperMotors[0].setControl(m_request);
    }

    public boolean hasReachedPos(double pos) {
        boolean hasReachedPos = (Math.abs(getHopperPos() - pos) <= HopperConstants.posTolerance);
        return hasReachedPos;
    }

    public void retractMotors() {
        moveHopperToPos(HopperConstants.retractedPos);
    }

    public void extendMotors() {
        moveHopperToPos(HopperConstants.extendedPos);
    }

    @Override
    public void periodic() {
        // find the motor's state and set them to their respective state
        findState();
    }
}
