package frc.robot.subsystems.SingleSolenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
public class SingleSolenoid extends SubsystemBase {
  private final SingleSolenoidIO Pneumatics;
  private final SingleSolenoidIOInputsAutoLogged inputs = new SingleSolenoidIOInputsAutoLogged();
  private final Solenoid pneumaticSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private final String name;

 
  public SingleSolenoid(SingleSolenoidIO io) {
    Pneumatics = io;

    name = io.getName();
    int channel = pneumaticSolenoid.getChannel();
    boolean isConnected = pneumaticSolenoid.get();
    
   
  }

  @Override
  public void periodic() {
    Pneumatics.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    }
  
  
  public void setMode(boolean isON) {
    pneumaticSolenoid.set(isON);
  }

}
