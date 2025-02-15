// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
/** Add your docs here. */
public class ButtonBoardButtons {
    public static final Trigger PNEUMATIC_OUT = RobotContainer.getButtonBoard().button(3);
    public static final Trigger PNEUMATIC_IN = RobotContainer.getButtonBoard().button(4);
    public static final Trigger UNLOAD_MANIPULATOR = RobotContainer.getButtonBoard().button(5);
    public static final Trigger LOAD_MANIPULATOR = RobotContainer.getButtonBoard().button(6);
    public static final Trigger LEVEL_1 = RobotContainer.getButtonBoard().button(7); 
    public static final Trigger LEVEL_2 = RobotContainer.getButtonBoard().button(8); 
    public static final Trigger LEVEL_3 = RobotContainer.getButtonBoard().button(9); 
    public static final Trigger LEVEL_4 = RobotContainer.getButtonBoard().button(10);
    public static final Trigger REEF_1 = RobotContainer.getButtonBoard().button(11); 
    public static final Trigger REEF_2 = RobotContainer.getButtonBoard().button(12); 
    public static final Trigger REEF_3 = RobotContainer.getButtonBoard().button(13); 
    public static final Trigger REEF_4 = RobotContainer.getButtonBoard().button(14);
    public static final Trigger REEF_5 = RobotContainer.getButtonBoard().button(15);
    public static final Trigger REEF_6 = RobotContainer.getButtonBoard().button(16);
     
}
