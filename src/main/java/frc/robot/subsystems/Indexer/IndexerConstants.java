package frc.robot.subsystems.Indexer;

public class IndexerConstants {
  public record IndexerGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record IndexerHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  public static final IndexerHardwareConfig EXAMPLE_CONFIG =
      new IndexerHardwareConfig(new int[] {11,12}, new boolean[] {false,true}, 2.0, 10, "");

  public static final IndexerHardwareConfig CompBot_CONFIG =
      new IndexerHardwareConfig(new int[] {11,12}, new boolean[] {true,true}, 20.0, 10, "");

  public static final IndexerGains SIM_GAINS =
      new IndexerGains(0.2, 0.0, 0, 0.0, 0.035, 0.0, 10.0, 0.2);
  public static final IndexerGains REAL_GAINS =
      new IndexerGains(0.1, 0.0, 0, 0.0, 0.3, 0.0, 30.0, 0.2);
}
