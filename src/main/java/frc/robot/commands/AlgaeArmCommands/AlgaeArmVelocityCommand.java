package frc.robot.commands.AlgaeArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;

import java.util.function.DoubleSupplier;

public class AlgaeArmVelocityCommand extends Command {
  private final AlgaeArm algaeArm;
  private final DoubleSupplier velocity;

  public AlgaeArmVelocityCommand(AlgaeArm algaeArm, DoubleSupplier velocity) {
    this.algaeArm = algaeArm;
    this.velocity = velocity;

    addRequirements(algaeArm);
  }

  @Override
  public void execute() {
    algaeArm.incrementPosition(velocity.getAsDouble() * 0.02);
  }
}
