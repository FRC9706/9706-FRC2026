package frc.robot.subsystems.doomAndDespair;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotContainer;
import lib.Networking.DynamicInputs;
import lib.Networking.DynamicInputs.DynamicChoice;
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
    private AudioConfigs audioConfig;
    private boolean isInitialized = false;

    private DynamicChoice trackChoice;

    public enum trackList {
        ussr,
        usa,
        test
    }
    private String[] trackNames;

    private boomBox() {
        mOrchestra = new Orchestra();

        audioConfig = new AudioConfigs();
        audioConfig.withAllowMusicDurDisable(true);

        // setup track names for dynamic input
        trackNames = convertEnumsToStrings();

        // create track choices
        createMusicChoice();
    }

    public void initOrchestra(TalonFX[] motorListInput) {
        if (isInitialized) {
            System.out.println("Orchestra already initialized, skipping...");
            return;
        }
        
        for (TalonFX motor : motorListInput) {
            mOrchestra.addInstrument(motor);        
        }
        
        isInitialized = true;
        System.out.println("Orchestra initialized with " + motorListInput.length + " motors");
    }

    public void loadAndPlayMusic(String choice, RobotContainer container) {
        // Lazy init: initialize on first play if not already done
        if (!isInitialized) {
            System.out.println("Orchestra not initialized yet, initializing now...");
            initOrchestra(getAllMotors(container));
        }
        
        String track = choice.concat(".chrp");
        var status = mOrchestra.loadMusic(track);
        if (!(status.isOK())) {
            System.err.println("Track loading failed! Attempted track: " + track + " Status: " + status.getDescription());
            return;
        }
        System.out.println("Loaded and playing track: " + track);
        mOrchestra.play();
    }

    public void stopMusic() {
        mOrchestra.stop();
    }

    public void createMusicChoice() {
        trackChoice = DynamicInputs.choice("boomBox/selectedTrack", 
        0, 
        trackNames, 
        chosen -> {
            System.out.println("Selected track: " + chosen);
        });
    }

    public String getMusicChoice() {
        return trackChoice.getSelected();
    }

    public String[] convertEnumsToStrings() {
        List<String> namesList = new ArrayList<>();

        for (trackList track : trackList.values()) {
            namesList.add(track.toString());
        }

        trackNames = namesList.toArray(new String[0]);

        return trackNames;
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
