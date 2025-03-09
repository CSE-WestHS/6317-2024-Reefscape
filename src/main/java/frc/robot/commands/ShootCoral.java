// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoral extends Command {
  double currentTime = 0;
  double startTime = 0;


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
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    if(Elevator.getPosition() < 0.3){
      Shooter.setVelocity(8);
    }
    else {
      Shooter.setVelocity(8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setVelocity(0);
    RobotContainer.hasShotCoral  = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentTime - startTime > 2){
      return true;
    }
    return Shooter.isFinished();//TODO: if second beambreak - change to stop when mid is not tripped
  }
}
