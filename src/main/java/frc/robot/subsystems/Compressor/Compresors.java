package frc.robot.subsystems.Compressor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Compressor.CompresorConstants.CompresorsGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
public class Compresors extends SubsystemBase {
  private final CompresorIO Compresor;
  private final CompresorsIOInputsAutoLogged inputs = new CompressorsIOInputsAutoLogged();
  private final Compressor pneumaticSolenoid = new Compressor(PneumaticsModuleType.REVPH);
  private final String name;

 
  public Compresors(CompresorIO io, CompresorsGains gains) {
    Compresor = io;

    name = io.getName();

   
  }

  @Override
  public void periodic() {
    Compresor.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    }

  // public void extend() {
  //   pneumaticSolenoid.set(true);
  // }

  // public void retract() {
  //   pneumaticSolenoid.set(false);
  // }

}
