package frc.robot.subsystems.Manipulator;

public class ManipulatorConstants {
  public record ManipulatorGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record ManipulatorHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final ManipulatorHardwareConfig EXAMPLE_CONFIG =
      new ManipulatorHardwareConfig(new int[] {30}, new boolean[] {false}, 24.0 / 48.0, "CANBus");

  public static final ManipulatorGains EXAMPLE_GAINS =
      new ManipulatorGains(5, 0.0, 0.0, 0.0, 0.065, 0.0);
}
