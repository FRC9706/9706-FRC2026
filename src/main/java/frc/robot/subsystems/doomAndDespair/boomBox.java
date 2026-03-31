package frc.robot.subsystems.doomAndDespair;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotContainer;
import swervelib.SwerveModule;

public class boomBox {
    private static boomBox mInstance = null;
    public static synchronized boomBox getInstance() {
        if (mInstance == null) {
            mInstance = new boomBox();
        }
        return mInstance;
    }

    private Orchestra mOrchestra;

    private enum trackList {
        test,
        hello
    }

    private boomBox() {
        mOrchestra = new Orchestra();
    }

    public void initOrchestra(TalonFX[] motorListInput) {
        for (TalonFX motor : motorListInput) {
            mOrchestra.addInstrument(motor);        
        }

        System.out.println("Orchestra initalize, motor list added: " + motorListInput);
    }

    public void loadAndPlayMusic(trackList choice) {
        String name = choice.toString();
        String track = name.concat(".chrp");
        var status = mOrchestra.loadMusic(track);
        if (!(status.isOK())) {
            System.out.print("track loading failed!");
            return;
        }
        System.out.print("Loaded track: " + track);
        mOrchestra.play();
    }

    public void stopMustic() {
        mOrchestra.stop();
    }

    public TalonFX[] getAllMotors(RobotContainer container) {
        List<TalonFX> motorList = new ArrayList<>();
        
        // *****************************************
        // Add Swerve Motor List
        // *****************************************

        // Get all swerve modules from the drivebase
        SwerveModule[] modules = container.getDrivebase().getSwerveDrive().getModules();
        
        // Iterate through each module and add its drive and angle motors
        for (SwerveModule module : modules) {
            // Access motors through configuration
            motorList.add((TalonFX) module.configuration.driveMotor.getMotor());
            motorList.add((TalonFX) module.configuration.angleMotor.getMotor());
        }

        // *****************************************
        // Build Hopper List
        // *****************************************
        TalonFX[] hopperMotors = container.getHopperInst().getHopperMotors();
        for (TalonFX motor : hopperMotors) {
            motorList.add(motor);
        }
        
        // Convert list to array
        return motorList.toArray(new TalonFX[0]);
    }
    
}
