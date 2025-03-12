package frc.robot.subsystems.AlgaeArm;

import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeArmConstants {
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

  public record AlgaeArmGains(
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
  public record AlgaeArmHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final AlgaeArmGains FunnelArm_GAINS =
      new AlgaeArmGains(0.05, 0.0, 5.6, 0.5, 1.0, 0.5, 0.0, 5.0, 2.0, 0.0, Math.PI, 0.73);

  public static final AlgaeArmHardwareConfig FunnelArm_CONFIG =
      new AlgaeArmHardwareConfig(
          new int[] {24},
          new boolean[] {false},
          1, 
          40,
          GravityType.COSINE,
          EncoderType.INTERNAL,
          45,
          Rotation2d.fromRadians(0),
          "");
}
