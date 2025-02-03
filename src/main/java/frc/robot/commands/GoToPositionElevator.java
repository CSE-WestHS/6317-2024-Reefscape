// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorGains;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToPositionElevator extends Command {
  Elevator Elevator;
  double position;
  ElevatorGains eg;
  /** Creates a new GoToPositionElevator. */
  public GoToPositionElevator(Elevator elevator, double desiredPosition, ElevatorGains EG) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Elevator = elevator;
    this.eg = EG;
    this.position = desiredPosition;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator.setPosition(position);
    
    System.out.println("Elevator GOING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((Elevator.getPosition() - eg.kMaxPosition()) >= 0) {
      return true;
    }
    if ((Elevator.getPosition() - eg.kMinPosition()) <= 0) {
      return true;
    }
    if (Math.abs(Elevator.getPosition() - position) >= eg.kTolerance()) {
      return true;
    }
    return false;
  }
}
