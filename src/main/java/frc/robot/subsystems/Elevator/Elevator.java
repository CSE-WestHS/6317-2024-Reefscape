package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorGains;
// import frc.robot.subsystems.flywheel.FlywheelIOInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableElevatorFeedforward;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO Elevator;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final String name;
  private TrapezoidProfile.State goal;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;
  // private final TunableElevatorFeedforward te = new TunableElevatorFeedforward();
  public Elevator(ElevatorIO io, ElevatorGains gains) {
    Elevator = io;
    

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
    Elevator.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          Elevator.setGains(
              new ElevatorGains(values[0], values[1], values[2], values[3], values[4], values[5]));
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA);
  }

  public void setPosition(double position, double desiredVelocity) {
    Elevator.setPosition(position, desiredVelocity);
  }

  public void setVelocity(double velocity) {
    Elevator.setVelocity(velocity);
  }

  public void setVoltage(double voltage) {
    Elevator.setVoltage(voltage);
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public double getVelocitySetpoint() {
    return inputs.desiredVelocity;
  }
}
