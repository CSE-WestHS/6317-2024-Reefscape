// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel.Funnel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FunnelUp extends Command {
  Funnel funnel;
  double currentTime;
  double startTime;
  /** Creates a new FunnelUp. */
  public FunnelUp(Funnel funnel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.funnel = funnel;
    addRequirements(funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   startTime = Timer.getFPGATimestamp();
    funnel.setVelocity(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    funnel.setVelocity(10);
    funnel.setMode(Value.kForward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnel.setVelocity(0);
    funnel.setMode(Value.kReverse);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentTime - startTime >=10) {
     return true; 
    }
    return false;
  }
}
