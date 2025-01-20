// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDChecker;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDS.LEDS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDChecker extends Command {
  private AddressableLED led;
  private AddressableLEDBuffer buff;
  private boolean lastcheck = false;
  LEDPattern red = LEDPattern.solid(Color.kRed);
  LEDPattern green = LEDPattern.solid(Color.kGreen);
    /** Creates a new LEDChecker. */
    public LEDChecker(AddressableLED led, AddressableLEDBuffer buff) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.led = led;
      this.buff = buff;
      

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (DriverStation.isDisabled()) {
              red.applyTo(buff);
          }
      if (DriverStation.isEnabled()) {
          green.applyTo(buff);
      }
      led.setData(buff);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
