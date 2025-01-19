package frc.robot.subsystems.Pneumatics;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Pneumatics.PneumaticsConstants.PneumaticsGains;

public interface PneumaticsIO {
  @AutoLog
  public static class PneumaticsIOInputs {
    public double velocity = 0.0;

    public double desiredVelocity = 0.0;

    public boolean[] motorsConnected = {false};

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(PneumaticsIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(PneumaticsGains gains) {}

  public default String getName() {
    return "Pneumatics";
  }
}
