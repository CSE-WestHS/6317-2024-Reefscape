package frc.robot.commands.position_joint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.function.DoubleSupplier;

public class PositionJointVelocityCommand extends Command {
  private final Elevator positionJoint;
  private final DoubleSupplier velocity;

  public PositionJointVelocityCommand(Elevator positionJoint, DoubleSupplier velocity) {
    this.positionJoint = positionJoint;
    this.velocity = velocity;

    addRequirements(positionJoint);
  }

  @Override
  public void execute() {
    positionJoint.incrementPosition(velocity.getAsDouble() * 0.02);
  }
}
