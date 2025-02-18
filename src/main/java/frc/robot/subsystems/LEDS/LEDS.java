// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.ButtonBoardButtons;

/** Add your docs here. */
public class LEDS {
    AddressableLED led;
    AddressableLEDBuffer ledbuffer;
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern cyan = LEDPattern.solid(Color.kDarkCyan);
    LEDPattern pink = LEDPattern.solid(Color.kPink);
    
    public LEDS(int length) {
        led = new AddressableLED(0);
        ledbuffer = new AddressableLEDBuffer(length);
        led.setLength(ledbuffer.getLength());
        

        led.setData(ledbuffer);
        led.start();
    }
    public void runLEDS() {
        if (ButtonBoardButtons.LEVEL_1.getAsBoolean()) {
            red.applyTo(ledbuffer);
            led.setData(ledbuffer);
        }
        if (ButtonBoardButtons.LEVEL_2.getAsBoolean()) {
            green.applyTo(ledbuffer);
            led.setData(ledbuffer);
        }
        if (ButtonBoardButtons.LEVEL_3.getAsBoolean()) {
            cyan.applyTo(ledbuffer);
            led.setData(ledbuffer);
        }
        if (ButtonBoardButtons.LEVEL_4.getAsBoolean()) {
            pink.applyTo(ledbuffer);
            led.setData(ledbuffer);
        }
        if (DriverStation.isDisabled()) {
            for (int i = 0; i < ledbuffer.getLength()/2; ++i) {
                ledbuffer.setLED(i, Color.kGreen);
            }
            
            for (int i = ledbuffer.getLength() / 2; i < ledbuffer.getLength(); ++i) {
                ledbuffer.setLED(i, Color.kCyan);
            }
            led.setData(ledbuffer);
        }
    }
}
