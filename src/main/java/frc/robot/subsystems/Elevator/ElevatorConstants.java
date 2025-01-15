package frc.robot.subsystems.Elevator;

public class ElevatorConstants {
  public record ElevatorGains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record ElevatorHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final ElevatorHardwareConfig EXAMPLE_CONFIG =
      new ElevatorHardwareConfig(new int[] {1}, new boolean[] {true}, 24.0 / 48.0, "CANBus");

  public static final ElevatorGains EXAMPLE_GAINS =
      new ElevatorGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0);
}
