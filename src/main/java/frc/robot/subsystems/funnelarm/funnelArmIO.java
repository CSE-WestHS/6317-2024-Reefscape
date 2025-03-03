package frc.robot.subsystems.funnelarm;

import frc.robot.subsystems.funnelarm.funnelArmConstants.funnelArmGains;

import org.littletonrobotics.junction.AutoLog;

public interface funnelArmIO {
  @AutoLog
  public static class funnelArmIOInputs {
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

  public default void updateInputs(funnelArmIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setPositon(double desiredPosition) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(funnelArmGains gains) {}

  public default String getName() { return "funnelArm"; };
}
