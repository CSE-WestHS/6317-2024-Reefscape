package frc.robot.subsystems.AlgaeArm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.AlgaeArm.AlgaeArmConstants.AlgaeArmGains;

public interface AlgaeArmIO {
  @AutoLog
  public static class AlgaeArmIOInputs {
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

  public default void updateInputs(AlgaeArmIOInputs inputs) {}

  public default void setPosition(double position, double velocity) {}

  public default void setGains(AlgaeArmGains gains) {}

  public default void setVoltage(double voltage) {}
  public default void setArmZero() {}
  public default String getName() {
    return "Position Joint";
  }
}
