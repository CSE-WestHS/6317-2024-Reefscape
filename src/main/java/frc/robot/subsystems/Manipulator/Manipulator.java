package frc.robot.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator.ManipulatorConstants.ManipulatorGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIO manipulator;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  public Manipulator(ManipulatorIO io, ManipulatorGains gains) {
    manipulator = io;

    name = io.getName();

    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());
  }

  @Override
  public void periodic() {
    manipulator.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          manipulator.setGains(
              new ManipulatorGains(values[0], values[1], values[2], values[3], values[4], values[5]));
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);
  }

  public void setVelocity(double velocity) {
    manipulator.setVelocity(velocity);
  }

  public void setVoltage(double voltage) {
    manipulator.setVoltage(voltage);
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getVelocitySetpoint() {
    return inputs.desiredVelocity;
  }
}
