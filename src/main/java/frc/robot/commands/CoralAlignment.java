// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.beam_break.BeamBreak;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralAlignment extends Command {
  //class variables
  Manipulator shooter;
  BeamBreak beamBreakMid;
  BeamBreak beamBreakBack;

  //beam break variables
  
  /** Creates a new CoralAlignment. */
  public CoralAlignment(Manipulator Shooter, BeamBreak BeamBreakMid, BeamBreak beamBreakBack) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = Shooter;
    this.beamBreakMid = BeamBreakMid;
    this.beamBreakBack = beamBreakBack;
    addRequirements(shooter, beamBreakBack);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
