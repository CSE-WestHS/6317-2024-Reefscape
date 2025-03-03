// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoral extends Command {
  Manipulator Shooter;
  Elevator Elevator;
  /** Creates a new ShootCoral. */
  public ShootCoral(Manipulator shooter, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Shooter = shooter;
    this.Elevator = elevator;
    addRequirements(shooter, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Elevator.getPosition() < 0.3){
      Shooter.setVelocity(1);
    }
    else {
      Shooter.setVelocity(3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
