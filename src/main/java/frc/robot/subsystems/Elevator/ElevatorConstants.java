package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorConstants {
  public enum GravityType {
    CONSTANT,
    COSINE,
    // Not supported by TalonFX
    SINE
  }

  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
  }

  public record ElevatorGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel,
      double kMinPosition,
      double kMaxPosition,
      double kTolerance) {}

  // Position Joint Gear Ratio should be multiplied by Math.PI * 2 for rotation joints to convert
  // from rotations to radians
  public record ElevatorHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final ElevatorGains EXAMPLE_GAINS =
      new ElevatorGains(1.5, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2);

  public static final ElevatorHardwareConfig EXAMPLE_CONFIG =
      new ElevatorHardwareConfig(
          new int[] {30,20},
          new boolean[] {false,true},
          85.33333 * 2 * Math.PI,
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          9999, //TODO: ID
          Rotation2d.fromRotations(0),
          "");
}
