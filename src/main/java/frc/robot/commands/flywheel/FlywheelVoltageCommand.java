package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import java.util.function.DoubleSupplier;

public class FlywheelVoltageCommand extends Command {
  private Elevator flywheel;
  private DoubleSupplier voltage;

  public FlywheelVoltageCommand(Elevator flywheel, DoubleSupplier voltage) {
    this.flywheel = flywheel;
    this.voltage = voltage;
    addRequirements(flywheel);
  }

  @Override
  public void execute() {
    flywheel.setVoltage(voltage.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
