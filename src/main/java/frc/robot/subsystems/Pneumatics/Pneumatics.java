package frc.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Pneumatics.PneumaticsConstants.PneumaticsGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
public class Pneumatics extends SubsystemBase {
  private final PneumaticsIO Pneumatics;
  private final PneumaticsIOInputsAutoLogged inputs = new PneumaticsIOInputsAutoLogged();
  private final Solenoid pneumaticSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private final String name;

 
  public Pneumatics(PneumaticsIO io, PneumaticsGains gains) {
    Pneumatics = io;

    name = io.getName();
    inputs.solenoid = pneumaticSolenoid.get();
    inputs.channel = pneumaticSolenoid.getChannel();
    
   
  }

  @Override
  public void periodic() {
    Pneumatics.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    }

  public void extend() {
    pneumaticSolenoid.set(true);
  }

  public void retract() {
    pneumaticSolenoid.set(false);
  }

}
