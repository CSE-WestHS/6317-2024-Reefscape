package frc.robot.subsystems.Funnel;

public class FunnelConstants {
  public record FunnelGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record FunnelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  public static final FunnelHardwareConfig EXAMPLE_CONFIG =
      new FunnelHardwareConfig(new int[] {41,42}, new boolean[] {false,true}, 2.0, 10, "");

  public static final FunnelGains SIM_GAINS =
      new FunnelGains(0.2, 0.0, 0, 0.0, 0.035, 0.0, 10.0, 0.2);
  public static final FunnelGains REAL_GAINS =
      new FunnelGains(1, 0.0, 0, 0.0, 0.5, 0.0, 1.0, 0.2);
}
