package frc.robot.subsystems.Compressor;

public class CompresorConstants {
  public record CompresorsGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record CompresorsHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final CompresorsHardwareConfig EXAMPLE_CONFIG =
      new CompresorsHardwareConfig(new int[] {1}, new boolean[] {true}, 24.0 / 48.0, "CANBus");

  public static final CompresorsGains EXAMPLE_GAINS =
      new CompresorsGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);
}
