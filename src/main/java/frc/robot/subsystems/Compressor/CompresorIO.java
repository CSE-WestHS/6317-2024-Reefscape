package frc.robot.subsystems.Compressor;

import org.littletonrobotics.junction.AutoLog;



public interface CompresorIO {
  @AutoLog
  public static class CompresorIOInputs {
    public boolean isDisabled = false;
    public double analogVoltage = 0.0;
    public double Current = 0.0;
    public boolean pressureSwitchValueReached = false;
  }

  public default void updateInputs(CompresorIOInputs inputs) {}

  public default void setAnalogVoltage(double voltage) {}
 
  public default void setCurrent(double Current) {}
 
  public default void setpreessureSwitchValueReached (boolean preesureSwitchValueReached) {} 
  
  public default void setisDisabled (boolean isDisabled) {}

  public default String getName() {
    return "Compresor";
  }
}
