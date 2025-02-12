// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator.Manipulator;

public class ManipulatorStart extends Command {
  private Manipulator Manip;
  private Timer clock;
  private double startTime;
  private double currentTime;
  /** Creates a new ManipulatorStart. */
  public ManipulatorStart(Manipulator Mani) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Mani);
    this.Manip = Mani;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Manip.setVelocity(1500);
    startTime = clock.getFPGATimestamp();
    
    //Manip.setVoltage(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Manip.setVelocity(1500);
    currentTime = clock.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Manip.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override

  public boolean isFinished() {
    if((currentTime-startTime)>=50){
      return true;
    }
    return false;
  }
}
