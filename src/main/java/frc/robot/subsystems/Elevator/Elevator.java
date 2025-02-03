package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorGains;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO elevator;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kG;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final LoggedTunableNumber kMaxVelo;
  private final LoggedTunableNumber kMaxAccel;

  private final LoggedTunableNumber kMinPosition;
  private final LoggedTunableNumber kMaxPosition;

  private final LoggedTunableNumber kTolerance;

  private final LoggedTunableNumber kSetpoint;

  private TrapezoidProfile.Constraints constraints;

  private TrapezoidProfile profile;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public Elevator(ElevatorIO io, ElevatorGains gains) {
    elevator = io;
    name = io.getName();

    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kG = new LoggedTunableNumber(name + "/Gains/kG", gains.kG());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());

    kMaxVelo = new LoggedTunableNumber(name + "/Gains/kMaxVelo", gains.kMaxVelo());
    kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());

    kMinPosition = new LoggedTunableNumber(name + "/Gains/kMinPosition", gains.kMinPosition());
    kMaxPosition = new LoggedTunableNumber(name + "/Gains/kMaxPosition", gains.kMaxPosition());

    kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());

    kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", getPosition());

    constraints = new TrapezoidProfile.Constraints(gains.kMaxVelo(), gains.kMaxAccel());
    profile = new TrapezoidProfile(constraints);

    goal = new TrapezoidProfile.State(getPosition(), 0);
    setpoint = goal;
  }

  @Override
  public void periodic() {
    elevator.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    setpoint = profile.calculate(0.02, setpoint, goal);

    elevator.setPosition(setpoint.position, setpoint.velocity);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          elevator.setGains(
              new ElevatorGains(
                  values[0],
                  values[1],
                  values[2],
                  values[3],
                  values[4],
                  values[5],
                  values[6],
                  values[7],
                  values[8],
                  values[9],
                  values[10],
                  values[11]));

          goal =
              new TrapezoidProfile.State(
                  MathUtil.clamp(values[12], kMinPosition.get(), kMaxPosition.get()), 0);

          constraints = new TrapezoidProfile.Constraints(values[7], values[8]);
          profile = new TrapezoidProfile(constraints);
        },
        kP,
        kI,
        kD,
        kS,
        kG,
        kV,
        kA,
        kMaxVelo,
        kMaxAccel,
        kMinPosition,
        kMaxPosition,
        kTolerance,
        kSetpoint);

    Logger.recordOutput(name + "/isFinished", isFinished());
  }

  public void setPosition(double position) {
    goal =
        new TrapezoidProfile.State(
            MathUtil.clamp(position, kMinPosition.get(), kMaxPosition.get()), 0);
  }

  public void incrementPosition(double deltaPosition) {
    goal.position += deltaPosition;
  }

  public void setVoltage(double voltage) {
    elevator.setVoltage(voltage);
  }

  public double getPosition() {
    return inputs.outputPosition;
  }

  public double getDesiredPosition() {
    return inputs.desiredPosition;
  }

  public boolean isFinished() {
    return Math.abs(inputs.outputPosition - goal.position) < kTolerance.get() && Math.abs(inputs.outputPosition - goal.position) > 0;
  }
}
