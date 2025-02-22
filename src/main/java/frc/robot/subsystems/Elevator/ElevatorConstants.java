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
      int currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final ElevatorGains EXAMPLE_GAINS =
      new ElevatorGains(0.1, 0.0, 0, 0.0, 0.08, .50, 0.01, 3.0, 1.0, 0.0, 30, 0.2);

  public static final ElevatorHardwareConfig EXAMPLE_CONFIG =
      new ElevatorHardwareConfig(
          new int[] {3,8},//{3,8}
          new boolean[] {false,true},
          1/( 2.8),//2.8 gives inches of travel //85.33333 * 2 * Math.PI
          20,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          9999, //TODO: ID
          Rotation2d.fromRotations(0),
          "");




  public static final ElevatorGains CompBot_GAINS =
      new ElevatorGains(0.1, 0.0, 0, 0.0, 0.08, .50, 0.01, 20.0, 30.0, 0.0, 30, 0.2);
    
  public static final ElevatorHardwareConfig CompBot_CONFIG =
      new ElevatorHardwareConfig(
          new int[] {3,8},//{3,8}
          new boolean[] {false,false},
          1/( 2.8),//2.8 gives inches of travel //85.33333 * 2 * Math.PI
          20,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          9999, //TODO: ID
          Rotation2d.fromRotations(0),
          "");
}
