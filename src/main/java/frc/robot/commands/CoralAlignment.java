// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.beam_break.BeamBreak;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralAlignment extends Command {
  //class variables
  private Manipulator shooter;
  private BeamBreak beamBreakMid;
  private BeamBreak beamBreakBack;
  
  //timer variables
  private double currentTime;
  private double startTime;

  //testing variables -- delete after beambreak is put on
  private boolean beamStateSim; //TODO: get rid of once robot is finished
  //beam break variables
  // private boolean BackState;
  // private boolean MidState;
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
    startTime = Timer.getFPGATimestamp();
    beamStateSim = beamBreakBack.beamBreakTripped(); //TODO: get rid of once robot is finished
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    if (currentTime - startTime > 5) {
      beamStateSim = false;//TODO: get rid of once robot is finished
      System.out.println("beamstate changed!");
    }
    if (beamBreakBack.beamBreakTripped() == true && beamStateSim == true) { //TODO: get rid of beamstatesim check once robot is finished
      shooter.setVelocity(10);
    }
    else if (beamStateSim == false) { shooter.setVelocity(0); } //TODO: get rid of once robot is finished
    else if ((beamBreakBack.beamBreakTripped() == false || beamStateSim == false) && beamBreakMid.beamBreakTripped() == true) {
      shooter.setVelocity(0);
    }
    else {
      shooter.setVelocity(0);
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
