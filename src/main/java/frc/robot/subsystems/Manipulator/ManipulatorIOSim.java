package frc.robot.subsystems.Manipulator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Manipulator.ManipulatorConstants.ManipulatorGains;
import frc.robot.subsystems.Manipulator.ManipulatorConstants.ManipulatorHardwareConfig;

public class ManipulatorIOSim implements ManipulatorIO {
  private final String name;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;

  private double velocitySetpoint;

  public ManipulatorIOSim(String name, ManipulatorHardwareConfig config) {
    this.name = name;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    gearBox = DCMotor.getNEO(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.025, config.gearRatio()), gearBox);

    controller = new PIDController(0.5, 0, 0);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    sim.setInputVoltage(controller.calculate(sim.getAngularVelocityRadPerSec(), velocitySetpoint));
    inputs.motorsConnected[0] = true;
    
    inputs.velocity = sim.getAngularVelocity().magnitude();
    inputs.desiredVelocity = velocitySetpoint;
  }

  @Override
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;
  }

  @Override
  public void setGains(ManipulatorGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());

    System.out.println(name + " gains set to " + gains);
  }
}
