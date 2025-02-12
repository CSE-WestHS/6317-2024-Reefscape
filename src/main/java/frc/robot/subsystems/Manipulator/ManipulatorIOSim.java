package frc.robot.subsystems.Manipulator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

  private boolean isConnected = false;
  
  private double gearRatio;

  private double inputVoltage;

  private ManipulatorHardwareConfig config;
  private boolean[] motorsConnected;
  private double[] motorPositions;
  private double[] motorVelocities;
  private double[] motorVoltages;
  private double[] motorCurrents;


  public ManipulatorIOSim(String name, ManipulatorHardwareConfig config) {
    this.name = name;
    this.config = config;
    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);
    isConnected = config.canIds().length > 0;
    gearRatio = config.gearRatio();
    gearBox = DCMotor.getNEO(/*config.canIds().length*/1);
    motorsConnected = new boolean[config.canIds().length];
    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.025, config.gearRatio()), gearBox);
;
    controller = new PIDController(0.5, 0, 0);
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    // inputVoltage = (controller.calculate(sim.getAngularVelocityRadPerSec(), velocitySetpoint));
    // // inputs.motorsConnected[0] = true;
    // inputs.motorVoltages[0] = inputVoltage;
    // sim.setAngularVelocity(inputs.desiredVelocity);
    // inputs.motorsConnected[0] = isConnected;
    // inputs.velocity = sim.getAngularVelocity().magnitude();
    // System.out.println("VELOCITY IS: " + inputs.velocity);
    // inputs.desiredVelocity = velocitySetpoint;
    inputVoltage = controller.calculate(sim.getAngularPosition().in(Rotations), velocitySetpoint);
    sim.setInputVoltage(inputVoltage);
    sim.update(0.02);

    
    inputs.velocity = sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < config.canIds().length; i++) {
      inputs.motorsConnected[i] = true;

      motorPositions[i] = sim.getAngularPosition().in(Rotations);
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond);
      motorVoltages[i] = sim.getInputVoltage();
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;

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
