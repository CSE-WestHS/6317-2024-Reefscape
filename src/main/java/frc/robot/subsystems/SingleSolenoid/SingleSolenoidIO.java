package frc.robot.subsystems.SingleSolenoid;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface SingleSolenoidIO {
  @AutoLog
  public static class SingleSolenoidIOInputs {
    public boolean isConnected = false; 
    public int channel = -1;
  }

  public default void updateInputs(SingleSolenoidIOInputs inputs) {}
  public default String getName() {
    return "Pneumatics";
  }
}
