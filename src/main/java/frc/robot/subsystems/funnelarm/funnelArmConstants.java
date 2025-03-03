package frc.robot.subsystems.funnelarm;

public class funnelArmConstants {
  public record funnelArmGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record funnelArmHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  public static final funnelArmHardwareConfig EXAMPLE_CONFIG =
      new funnelArmHardwareConfig(new int[] {13}, new boolean[] {false}, 2.0, 10, "");

  public static final funnelArmHardwareConfig CompBot_CONFIG =
      new funnelArmHardwareConfig(new int[] {13}, new boolean[] {true}, 12.0, 10, "");

  public static final funnelArmGains SIM_GAINS =
      new funnelArmGains(0.2, 0.0, 0, 0.0, 0.035, 0.0, 10.0, 0.2);
  public static final funnelArmGains REAL_GAINS =
      new funnelArmGains(0.1, 0.0, 0, 0.0, 0.3, 0.0, 30.0, 0.2);
}
