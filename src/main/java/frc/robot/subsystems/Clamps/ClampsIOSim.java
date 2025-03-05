package frc.robot.subsystems.Clamps;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Clamps.ClampsConstants.ClampsGains;
import frc.robot.subsystems.Clamps.ClampsConstants.ClampsHardwareConfig;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;

public class ClampsIOSim implements ClampsIO {
  private final String name;

  private final ClampsHardwareConfig config;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;
  private final TunableSimpleMotorFeedforward feedforward;

  private final double[] motorPositions;
  private final double[] motorVelocities;
  private final double[] motorAccelerations;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private double velocitySetpoint = 0;
  private boolean isConnected = false; 
  public ClampsIOSim(String name, ClampsHardwareConfig config) {
    this.name = name;

    this.config = config;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length); //check if the simulation is valid

    isConnected = config.canIds().length > 0; //check if motors are on by seeing if canids exist
    
    //set logging arrays to store values for each motor: therefore, make each array the size of the amount of motors
    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorAccelerations = new double[config.canIds().length];
    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];

    //set up simulation for NEO motor
    gearBox = DCMotor.getNEO(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.025, config.gearRatio()), gearBox);

    //set up PID
    controller = new PIDController(0, 0, 0);
    feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);
    setGains(ClampsConstants.SIM_GAINS); //sets sim gains for simulation
  }
  @Override
  public void updateInputs(ClampsIOInputs inputs) {
    inputs.motorsConnected[0] = isConnected; //check if motors are connected
    //drive the motor in simulation then store logging values every 0.02 seconds
    double inputVoltage =
        controller.calculate(sim.getAngularVelocityRPM(), velocitySetpoint)
            + feedforward.calculateWithVelocities(sim.getAngularVelocityRPM(), velocitySetpoint);
    sim.setInputVoltage(inputVoltage);
    sim.update(0.02);

    inputs.velocity = sim.getAngularVelocityRPM();
    inputs.desiredVelocity = velocitySetpoint;
    // System.out.println("Current Velocity: " + inputs.velocity + "\nWanted Velocity: " + inputs.desiredVelocity);
    for (int i = 0; i < config.canIds().length; i++) {
      motorPositions[i] = sim.getAngularPositionRotations();
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond)*60;
      motorAccelerations[i] = sim.getAngularAcceleration().in(RotationsPerSecondPerSecond) * 60;

      motorVoltages[i] = inputVoltage;
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;
    inputs.motorAccelerations = motorAccelerations;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;
  }

  @Override
  public void setGains(ClampsGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());
    feedforward.setGains(gains.kS(), gains.kV(), gains.kA());

    System.out.println(name + " gains set to " + gains);
  }

  @Override
  public String getName() {
    return name;
  }
}
