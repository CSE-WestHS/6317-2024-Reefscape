package frc.robot.subsystems.Compressor;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Compressor.CompresorConstants.CompresorsGains;


public interface CompresorIO {
  @AutoLog
  public static class CompresorIOInputs {
    public double velocity = 0.0;

    public double desiredVelocity = 0.0;

    public boolean[] motorsConnected = {false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(CompresorIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(CompresorsGains gains) {}

  public default String getName() {
    return "Compresor";
  }
}
