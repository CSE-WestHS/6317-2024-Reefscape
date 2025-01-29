package frc.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class Pneumatics extends SubsystemBase {
  private final PneumaticsIO Pneumatics;
  private final PneumaticsIOInputsAutoLogged inputs = new PneumaticsIOInputsAutoLogged();
  private final DoubleSolenoid pneumaticSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7,8);
  private final String name;

 
  public Pneumatics(PneumaticsIO io) {
    Pneumatics = io;

    name = io.getName();
    inputs.solenoid = pneumaticSolenoid.get();
    inputs.channelForward = pneumaticSolenoid.getFwdChannel();
    inputs.channelReverse = pneumaticSolenoid.getRevChannel();
    
   
  }

  @Override
  public void periodic() {
    Pneumatics.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    }
  
  public void setSolenoid(Value mode) { inputs.solenoid = mode; }
  
  public void setChannelForward(int channel) { inputs.channelForward = channel; }
  
  public void setChannelReverse(int channel) { inputs.channelReverse = channel; }
  
  public void setMode(Value mode) {
    pneumaticSolenoid.set(mode);
    setSolenoid(mode);
  }

}
