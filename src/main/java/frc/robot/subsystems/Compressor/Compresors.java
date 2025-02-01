package frc.robot.subsystems.Compressor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class Compresors extends SubsystemBase {
  private final CompresorIO Compresor;
  private final CompresorIOInputsAutoLogged inputs = new CompresorIOInputsAutoLogged();
  private final Compressor compressor1 = new Compressor(PneumaticsModuleType.REVPH);
  private final PneumaticHub pneumaticHub = new PneumaticHub();
  private final String name;
  private boolean disable;

 
  public Compresors(CompresorIO io, boolean disable) {
    Compresor = io;

    name = io.getName();
    
    this.disable = disable;

   
  }

  @Override
  public void periodic() {
    if (disable == false) {
      compressor1.enableDigital();
    }
    else {
      compressor1.disable();
    }
    inputs.isDisabled = !compressor1.isEnabled();
    inputs.Current = compressor1.getCurrent();
    inputs.analogVoltage = compressor1.getAnalogVoltage();
    inputs.pressureSwitchValueReached = !compressor1.getPressureSwitchValue();
    inputs.pressure = pneumaticHub.getPressure(0);
    Compresor.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    }

    public void setDisable(boolean disable) {
      this.disable = disable;
    }

}
