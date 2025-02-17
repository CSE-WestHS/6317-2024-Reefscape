// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;

import java.util.Map;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class LEDS {
    AddressableLED led;
    AddressableLEDBuffer ledbuffer;
    AddressableLEDBuffer buff2;
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern f = LEDPattern.rainbow(150, 150);
    LEDPattern cyan = LEDPattern.solid(Color.kDarkCyan);
    
    public LEDS(int length) {
        led = new AddressableLED(0);
        ledbuffer = new AddressableLEDBuffer(length);
        led.setLength(ledbuffer.getLength());
        // buff2 = new AddressableLEDBuffer(length/2);
        

        led.setData(ledbuffer);
        led.start();
    }
    public void runLEDS() {
        if (DriverStation.isEStopped()) {
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
