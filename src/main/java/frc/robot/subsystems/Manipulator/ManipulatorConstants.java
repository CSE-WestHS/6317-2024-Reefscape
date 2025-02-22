package frc.robot.subsystems.Manipulator;

public class ManipulatorConstants {
  public record ManipulatorGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record ManipulatorHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final ManipulatorHardwareConfig EXAMPLE_CONFIG =
      new ManipulatorHardwareConfig(new int[] {14}, new boolean[] {true}, 2.0, "");

  public static final ManipulatorGains SIM_GAINS =
      new ManipulatorGains(0.6, 0.0, 0, 0.0, 0.035, 0.0, 10.0, 0.2);
  public static final ManipulatorGains REAL_GAINS =
      new ManipulatorGains(1, 0.0, 0, 0.0, 0.5, 0.0, 1.0, 0.2);
}
