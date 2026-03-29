package lib.Networking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class DynamicInputs {

  private static final String rootTableString = "/DynamicInputs";

  private static final DynamicInputs instance = new DynamicInputs();
  private static final AtomicInteger idGenerator = new AtomicInteger();

  private final Map<Integer, dynamicNum> tunables = new HashMap<>();

  // CRITICAL! Change this value to enable/disable LIVE tuning!
  private static final boolean inputsEnabled = true;

  private DynamicInputs() {}

  // Call once per loop (Robot's periodic is ideal)
  public static void periodic() {
    instance.update();
  }

  public static dynamicNum number(String key, double defaultValue) {
    return instance.createNumber(key, defaultValue);
  }

  // Create a dynamic input box for whate
  public static void DynamicNum(String name, double defaultValue, numConsumer consumer) {
    number(name, defaultValue).onChange(consumer::accept);
  }

  // Create a tunable PID set
  public static void pid(
      String name,
      double defaultP,
      double defaultI,
      double defaultD,
      PIDConsumer consumer
  ) {
    dynamicNum kP = number(name + "/kP", defaultP);
    dynamicNum kI = number(name + "/kI", defaultI);
    dynamicNum kD = number(name + "/kD", defaultD);

    Runnable apply = () -> consumer.accept(kP.get(), kI.get(), kD.get());

    kP.onChange(v -> apply.run());
    kI.onChange(v -> apply.run());
    kD.onChange(v -> apply.run());
  }

  // Create a tunable PIDSVA + magic motion parameters set
  public static void pidMagicMotion (
      String name,
      double defaultP,
      double defaultI,
      double defaultD,
      double defaultS,
      double defaultV,
      double defaultA,
      double defaultCruiseVel,
      double defaultAccel,
      double defaultJerk,
      PIDMagicMotionConsumer consumer) {

    dynamicNum kP = number(name + "/kP", defaultP);
    dynamicNum kI = number(name + "/kI", defaultI);
    dynamicNum kD = number(name + "/kD", defaultD);
    dynamicNum kS = number(name + "/kS", defaultS);
    dynamicNum kV = number(name + "/kV", defaultV);
    dynamicNum kA = number(name + "/kA", defaultA);
    dynamicNum cruiseVel = number(name + "/MotionMagicCruiseVel", defaultCruiseVel);
    dynamicNum accel = number(name + "/MotionMagicAccel", defaultAccel);
    dynamicNum jerk = number(name + "/MotionMagicJerk", defaultJerk);

    Runnable apply = () -> consumer.accept(
    kP.get(), kI.get(), kD.get(), 
    kS.get(), kV.get(), kA.get(), 
    cruiseVel.get(), accel.get(), jerk.get());

    kP.onChange(v -> apply.run());
    kI.onChange(v -> apply.run());
    kD.onChange(v -> apply.run());
    kS.onChange(v -> apply.run());
    kV.onChange(v -> apply.run());
    kA.onChange(v -> apply.run());
    cruiseVel.onChange(v -> apply.run());
    accel.onChange(v -> apply.run());
    jerk.onChange(v -> apply.run());
  }

  /**
   * Create a dynamic dropdown/choice selector in Shuffleboard.
   * When the selection changes, the corresponding option's callback is executed.
   * 
   * @param name Name of the choice widget in Shuffleboard (e.g., "PortForwarder/LL")
   * @param defaultChoice Index of the default selected option (0-based)
   * @param options Array of option names to display in dropdown (use a new String[])
   * @param consumer Callback that receives the index of selected option
   */
  
  public static DynamicChoice choice (
      String name,
      int defaultChoice,
      String[] options,
      Consumer<Integer> consumer) {
    DynamicChoice choice = new DynamicChoice(name, defaultChoice, options, consumer);
    instance.registerChoice(choice);
    return choice;
  }

  private final Map<Integer, DynamicChoice> choices = new HashMap<>();

  private void registerChoice(DynamicChoice choice) {
    int id = idGenerator.incrementAndGet();
    choices.put(id, choice);
  }

  private dynamicNum createNumber(String key, double defaultValue) {
    int id = idGenerator.incrementAndGet(); // kept only for map keys
    dynamicNum number = new dynamicNum(key, defaultValue);
    tunables.put(id, number);
    return number;
  }

  private void update() {
    tunables.values().forEach(dynamicNum::update);
    choices.values().forEach(DynamicChoice::update);
  }

  // ---------------------------------------------------------------------------
  // Dynamic Number
  // ---------------------------------------------------------------------------

  public static final class dynamicNum implements DoubleSupplier {

    private final String fullKey;
    private final double defaultValue;

    private final List<DoubleConsumer> changeCallbacks = new ArrayList<>();

    private LoggedNetworkNumber networkNumber;
    private double currentValue;
    private boolean firstUpdate = true;

    private dynamicNum(String key, double defaultValue) {
      this.fullKey = rootTableString + "/" + key;
      this.defaultValue = defaultValue;

      if (inputsEnabled) {
        networkNumber = new LoggedNetworkNumber(fullKey, defaultValue);
      }

      currentValue = defaultValue;
    }

    private void update() {
      double newValue = inputsEnabled
          ? networkNumber.get()
          : defaultValue;

      if (firstUpdate || newValue != currentValue) {
        firstUpdate = false;
        currentValue = newValue;

        for (DoubleConsumer callback : changeCallbacks) {
          callback.accept(currentValue);
        }
      }
    }

    // Run something automatically when the value changes
    public dynamicNum onChange(DoubleConsumer consumer) {
      changeCallbacks.add(consumer);
      consumer.accept(currentValue);
      return this;
    }

    public double get() {
      return currentValue;
    }

    @Override
    public double getAsDouble() {
      return get();
    }
  }

  // ---------------------------------------------------------------------------
  // Tunable Choice
  // ---------------------------------------------------------------------------

  public static final class DynamicChoice {

    private final String[] options;
    private final LoggedDashboardChooser<Integer> chooser;
    private int lastIndex;

    private DynamicChoice(String key, int defaultChoice, String[] options, Consumer<Integer> consumer) {
      this.options = options;
      this.lastIndex = defaultChoice;

      if (inputsEnabled) {
        // Create LoggedDashboardChooser with Integer values
        chooser = new LoggedDashboardChooser<>(key);
        
        // Add default option
        chooser.addDefaultOption(options[defaultChoice], defaultChoice);
        
        // Add remaining options
        for (int i = 0; i < options.length; i++) {
          if (i != defaultChoice) {
            chooser.addOption(options[i], i);
          }
        }

        // Set callback for when selection changes
        chooser.onChange(selectedIndex -> {
          lastIndex = selectedIndex;
          consumer.accept(selectedIndex);
        });
      }
    }

    private void update() {
      // LoggedDashboardChooser handles its own updates via periodic()
      // which is called by AdvantageKit's Logger
    }

    public int get() {
      return lastIndex;
    }
    
    public String getSelected() {
      return options[lastIndex];
    }
  }

  // ---------------------------------------------------------------------------
  // Functional Interfaces
  // ---------------------------------------------------------------------------

  @FunctionalInterface
  public interface numConsumer {
    void accept(double value);
  }

  public interface PIDConsumer {
    void accept(double kP, double kI, double kD);
  }

  public interface PIDMagicMotionConsumer {
    void accept(
      double kP, double kI, double kD,
      double kS, double kV, double kA,
      double cruiseVel, double accel, double jerk
    );
  }
}
