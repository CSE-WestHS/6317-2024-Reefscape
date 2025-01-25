// package frc.robot.commands.flywheel;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator.Elevator;
// import java.util.function.DoubleSupplier;

// public class FlywheelVelocityCommand extends Command {
//   private Elevator flywheel;
//   private DoubleSupplier velocity;

//   public FlywheelVelocityCommand(Elevator flywheel, DoubleSupplier velocity) {
//     this.flywheel = flywheel;
//     this.velocity = velocity;
//     addRequirements(flywheel);
//   }

//   @Override
//   public void initialize() {
//     flywheel.setVelocity(velocity.getAsDouble());
//   }

//   @Override
//   public void execute() {
//     flywheel.setVelocity(velocity.getAsDouble());
//   }

//   @Override
//   public boolean isFinished() {
//     return Math.abs(flywheel.getVelocity() - velocity.getAsDouble()) < 1.5;
//   }
// }
