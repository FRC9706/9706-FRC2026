package frc.robot.util.Tuning;

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

public final class LiveTuner {

  private static final String rootTableString = "/Tuning";

  private static final LiveTuner instance = new LiveTuner();
  private static final AtomicInteger idGenerator = new AtomicInteger();

  private final Map<Integer, TunableNumber> tunables = new HashMap<>();

  // CRITICAL! Change this value to enable/disable LIVE tuning!
  private static final boolean tunningEnabled = true;

  private LiveTuner() {}

  // Call once per loop (Robot's periodic is ideal)
  public static void periodic() {
    instance.update();
  }

  public static TunableNumber number(String key, double defaultValue) {
    return instance.createNumber(key, defaultValue);
  }

  // Create a tunable PID set
  public static void pid(
      String name,
      double defaultP,
      double defaultI,
      double defaultD,
      PIDConsumer consumer
  ) {
    TunableNumber kP = number(name + "/kP", defaultP);
    TunableNumber kI = number(name + "/kI", defaultI);
    TunableNumber kD = number(name + "/kD", defaultD);

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
      PIDMagicMotionConsumer consumer
  ) {
    TunableNumber kP = number(name + "/kP", defaultP);
    TunableNumber kI = number(name + "/kI", defaultI);
    TunableNumber kD = number(name + "/kD", defaultD);
    TunableNumber kS = number(name + "/kS", defaultS);
    TunableNumber kV = number(name + "/kV", defaultV);
    TunableNumber kA = number(name + "/kA", defaultA);
    TunableNumber cruiseVel = number(name + "/MotionMagicCruiseVel", defaultCruiseVel);
    TunableNumber accel = number(name + "/MotionMagicAccel", defaultAccel);
    TunableNumber jerk = number(name + "/MotionMagicJerk", defaultJerk);

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
   * Create a tunable dropdown/choice selector in Shuffleboard.
   * When the selection changes, the corresponding option's callback is executed.
   * 
   * @param name Name of the choice widget in Shuffleboard (e.g., "PortForwarder/LL")
   * @param defaultChoice Index of the default selected option (0-based)
   * @param options Array of option names to display in dropdown (use a new String[])
   * @param consumer Callback that receives the index of selected option
   */
  
  public static void choice(
      String name,
      int defaultChoice,
      String[] options,
      Consumer<Integer> consumer) {
    TunableChoice choice = new TunableChoice(name, defaultChoice, options, consumer);
    instance.registerChoice(choice);
  }

  private final Map<Integer, TunableChoice> choices = new HashMap<>();

  private void registerChoice(TunableChoice choice) {
    int id = idGenerator.incrementAndGet();
    choices.put(id, choice);
  }

  private TunableNumber createNumber(String key, double defaultValue) {
    int id = idGenerator.incrementAndGet(); // kept only for map keys
    TunableNumber number = new TunableNumber(key, defaultValue);
    tunables.put(id, number);
    return number;
  }

  private void update() {
    tunables.values().forEach(TunableNumber::update);
    choices.values().forEach(TunableChoice::update);
  }

  // ---------------------------------------------------------------------------
  // Tunable Number
  // ---------------------------------------------------------------------------

  public static final class TunableNumber implements DoubleSupplier {

    private final String fullKey;
    private final double defaultValue;

    private final List<DoubleConsumer> changeCallbacks = new ArrayList<>();

    private LoggedNetworkNumber networkNumber;
    private double currentValue;
    private boolean firstUpdate = true;

    private TunableNumber(String key, double defaultValue) {
      this.fullKey = rootTableString + "/" + key;
      this.defaultValue = defaultValue;

      if (tunningEnabled) {
        networkNumber = new LoggedNetworkNumber(fullKey, defaultValue);
      }

      currentValue = defaultValue;
    }

    private void update() {
      double newValue = tunningEnabled
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
    public TunableNumber onChange(DoubleConsumer consumer) {
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

  public static final class TunableChoice {

    private final String[] options;
    private final LoggedDashboardChooser<Integer> chooser;
    private int lastIndex;

    private TunableChoice(String key, int defaultChoice, String[] options, Consumer<Integer> consumer) {
      this.options = options;
      this.lastIndex = defaultChoice;

      if (tunningEnabled) {
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
