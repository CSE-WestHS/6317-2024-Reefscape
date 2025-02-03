package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorGains;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double outputPosition = 0.0;
    public double rotorPosition = 0.0;
    public double desiredPosition = 0.0;

    public double velocity = 0.0;
    public double desiredVelocity = 0.0;

    public boolean[] motorsConnected = {false};
    public boolean encoderConnected = false;

    public double[] motorPositions = {0.0};
    public double[] motorVelocities = {0.0};
    public double[] motorAccelerations = {0.0};

    public double[] motorVoltages = {0.0};
    public double[] motorCurrents = {0.0};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double position, double velocity) {}

  public default void setGains(ElevatorGains gains) {}

  public default void setVoltage(double voltage) {}

  public default String getName() {
    return "Elevator";
  }
}
