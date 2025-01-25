package frc.robot.subsystems.Pneumatics;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface PneumaticsIO {
  @AutoLog
  public static class PneumaticsIOInputs {
    public Value solenoid = Value.kOff; 
    public int channelForward = -1;
    public int channelReverse = -1;
  }

  public default void updateInputs(PneumaticsIOInputs inputs) {}
  public default void setSolenoid(Value mode) {}
  public default void setChannelForward(int channel) {}
  public default void setChannelReverse(int channel) {}
  public default String getName() {
    return "Pneumatics";
  }
}
