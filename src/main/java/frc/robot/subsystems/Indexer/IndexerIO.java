package frc.robot.subsystems.Indexer;

import frc.robot.subsystems.Indexer.IndexerConstants.IndexerGains;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double velocity = 0.0;
    public double desiredVelocity = 0.0;

    public double position = 0.0;
    public double desiredPosition = 0.0;

    public boolean[] motorsConnected = {false,false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setPositon(double desiredPosition) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(IndexerGains gains) {}

  public default String getName() { return "Indexer"; };
}
