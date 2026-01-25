package frc.robot.util.Tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class LiveTuner {

  private static final String ROOT_TABLE = "/Tuning";

  private static final LiveTuner instance = new LiveTuner();
  private static final AtomicInteger idGenerator = new AtomicInteger();

  private final Map<Integer, TunableNumber> tunables = new HashMap<>();

  private LiveTuner() {}

  // Call once per loop (robotPeriodic is ideal)
  public static void periodic() {
    instance.update();
  }

  //
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

  private TunableNumber createNumber(String key, double defaultValue) {
    int id = idGenerator.incrementAndGet(); // kept only for map keys
    TunableNumber number = new TunableNumber(key, defaultValue);
    tunables.put(id, number);
    return number;
  }

  private void update() {
    tunables.values().forEach(TunableNumber::update);
  }

  private static boolean tuningEnabled() {
    return RobotBase.isSimulation() || DriverStation.isTest();
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
      this.fullKey = ROOT_TABLE + "/" + key;
      this.defaultValue = defaultValue;

      if (tuningEnabled()) {
        networkNumber = new LoggedNetworkNumber(fullKey, defaultValue);
      }

      currentValue = defaultValue;
    }

    private void update() {
      double newValue = tuningEnabled()
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
}
