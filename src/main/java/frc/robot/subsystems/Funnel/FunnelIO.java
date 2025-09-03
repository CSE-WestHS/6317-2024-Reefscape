package frc.robot.subsystems.Funnel;

import frc.robot.subsystems.Funnel.FunnelConstants.FunnelGains;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
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
    public Value solenoid = Value.kReverse; 
    public int channelForward = -1;
    public int channelReverse = -1;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setVelocity(double velocity) {}

  public default void setPositon(double desiredPosition) {}

  public default void setVoltage(double voltage) {}

  public default void setGains(FunnelGains gains) {}
  
  public default void setSolenoid(Value mode) {}
  
  public default void setChannelForward(int channel) {}
  
  public default void setChannelReverse(int channel) {}

  public default String getName() { return "Funnel"; };
}
