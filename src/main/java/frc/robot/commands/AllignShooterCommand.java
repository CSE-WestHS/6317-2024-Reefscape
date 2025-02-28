// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.beam_break.BeamBreak;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AllignShooterCommand extends Command {
  BeamBreak beamBreak;
  Manipulator shooter;
  /** Creates a new ShooterCommand. */
  public AllignShooterCommand(Manipulator Shooter, BeamBreak BeamBreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.beamBreak = BeamBreak;
    this.shooter = Shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.beamBreakTripped() == false) {
      shooter.setVoltage(0);
    }
    else if (beamBreak.beamBreakTripped() == true) {
      shooter.setVelocity(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
