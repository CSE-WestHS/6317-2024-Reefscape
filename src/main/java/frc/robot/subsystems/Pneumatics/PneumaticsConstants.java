package frc.robot.subsystems.Pneumatics;

public class PneumaticsConstants {
  public record PneumaticsGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record PneumaticsHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final PneumaticsHardwareConfig EXAMPLE_CONFIG =
      new PneumaticsHardwareConfig(new int[] {1}, new boolean[] {true}, 24.0 / 48.0, "CANBus");

  public static final PneumaticsGains EXAMPLE_GAINS =
      new PneumaticsGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);
}
