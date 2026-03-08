package frc.robot.util.Tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class LiveTuner {

  private static final String rootTableString = "/Tuning";

  private static final LiveTuner instance = new LiveTuner();
  private static final AtomicInteger idGenerator = new AtomicInteger();

  private final Map<Integer, TunableNumber> tunables = new HashMap<>();

  // CRITICAL! Change this value to enable/disable LIVE tuning!
  private static final boolean tunningEnabled = true;

  private LiveTuner() {}

  // Call once per loop (robotPeriodic is ideal)
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

  private TunableNumber createNumber(String key, double defaultValue) {
    int id = idGenerator.incrementAndGet(); // kept only for map keys
    TunableNumber number = new TunableNumber(key, defaultValue);
    tunables.put(id, number);
    return number;
  }

  private void update() {
    tunables.values().forEach(TunableNumber::update);
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
