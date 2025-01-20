package frc.robot.subsystems.Pneumatics;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Pneumatics.PneumaticsConstants.PneumaticsGains;

public interface PneumaticsIO {
  @AutoLog
  public static class PneumaticsIOInputs {
    public boolean solenoid = false;
    public int channel = 0;
  }

  public default void updateInputs(PneumaticsIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(PneumaticsGains gains) {}

  public default String getName() {
    return "Pneumatics";
  }
}
