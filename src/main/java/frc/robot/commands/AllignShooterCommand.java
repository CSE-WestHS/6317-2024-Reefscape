// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.beam_break.BeamBreak;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AllignShooterCommand extends Command {
  BeamBreak beamBreak;
  Manipulator shooter;
  Indexer indexer;
  public boolean seenCoral = false;
  public boolean commandDone = false;
  /** Creates a new ShooterCommand. */
  public AllignShooterCommand(Manipulator Shooter, BeamBreak BeamBreak,Indexer Indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.beamBreak = BeamBreak;
    this.shooter = Shooter;
    this.indexer = Indexer;
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setVoltage(6);
    seenCoral = false;
    commandDone = false;
    System.out.println("Initialized");
    // while (!beamBreak.beamBreakTripped()) {
    //   indexer.setVoltage(6);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.beamBreakTripped() == true){
      seenCoral = true;
      System.out.println("Coral seen");
    }

    if(seenCoral){
      shooter.setVoltage(6);
      indexer.setVoltage(3);
      System.out.println("shooter and indexer");

      if(beamBreak.beamBreakTripped()==false){
        shooter.setVoltage(0);
        indexer.setVoltage(0);
        commandDone = true;
        System.out.println("command done");
      }
    }
    else{
      indexer.setVoltage(6);
      System.out.println("else statement");
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(0);
    indexer.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
