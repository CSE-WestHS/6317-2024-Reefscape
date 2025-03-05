package frc.robot.subsystems.Clamps;

public class ClampsConstants {
  public record ClampsGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record ClampsHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  public static final ClampsHardwareConfig EXAMPLE_CONFIG =
      new ClampsHardwareConfig(new int[] {14}, new boolean[] {true}, 2.0,10, "");

  public static final ClampsHardwareConfig CompBot_CONFIG =
      new ClampsHardwareConfig(new int[] {14}, new boolean[] {false}, 3.8, 10, "");

  public static final ClampsGains SIM_GAINS =
      new ClampsGains(0.6, 0.0, 0, 0.0, 0.035, 0.0, 10.0, 0.2);


  public static final ClampsGains REAL_GAINS =
      new ClampsGains(0.01, 0.0, 0, 0.0, 3, 0.0, 150.0, 0.2);
}
