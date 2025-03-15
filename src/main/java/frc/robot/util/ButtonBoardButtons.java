// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
/** Add your docs here. */
public class ButtonBoardButtons {

    /*
     * 
     * private Trigger forwardX = new Trigger(()->varJoystick.getX() > 0.5);
  private Trigger BackwardX = new Trigger(()->varJoystick.getX() < -0.5);
  private Trigger forwardY = new Trigger(()->varJoystick.getY() > 0.5);
  private Trigger BackwardY = new Trigger(()->varJoystick.getY() < -0.5);
     * 
     * 
     * 
     */
    public static final Trigger LEVEL_4 = RobotContainer.getButtonBoard().button(7); 
    public static final Trigger LEVEL_3 = RobotContainer.getButtonBoard().button(8); 
    public static final Trigger LEVEL_2 = RobotContainer.getButtonBoard().button(9); 
    public static final Trigger LEVEL_1 = RobotContainer.getButtonBoard().button(10);
    public static final Trigger L1 = RobotContainer.getButtonBoard().button(3);
    public static final Trigger R3 = RobotContainer.getButtonBoard().button(6);
    public static final Trigger R2 = RobotContainer.getButtonBoard().button(5);
    public static final Trigger R5 = RobotContainer.getButtonBoard().button(12);
    public static final Trigger R4 = RobotContainer.getButtonBoard().button(11);
    public static final Trigger L6 = new Trigger(()->RobotContainer.getButtonBoard().getX() > 0.5); //FX
    public static final Trigger R6 = new Trigger(()->RobotContainer.getButtonBoard().getX() > -0.5); //BX
    public static final Trigger L4 = new Trigger(()->RobotContainer.getButtonBoard().getY() > -0.5); //BY
    public static final Trigger L5 = new Trigger(()->RobotContainer.getButtonBoard().getY() > 0.5); //FY
    public static final Trigger L2 = RobotContainer.getButtonBoard().button(2);
    public static final Trigger L3 = RobotContainer.getButtonBoard().button(1);
    public static final Trigger R1 = RobotContainer.getButtonBoard().button(4);
     
}
