package frc.robot.commands.AlgaeArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;

public class AlgaeArmPositionCommand extends Command {
  private final AlgaeArm algaeArm;
  private final double position;

  public AlgaeArmPositionCommand(AlgaeArm algaeArm, double position) {
    this.algaeArm = algaeArm;
    this.position = position;

    addRequirements(algaeArm);
  }

  @Override
  public void initialize() {
    algaeArm.setPosition(position);
  }

  @Override
  public boolean isFinished() {
    return algaeArm.isFinished();
  }
}
