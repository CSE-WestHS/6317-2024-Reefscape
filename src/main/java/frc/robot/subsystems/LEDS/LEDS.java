// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDS {
    AddressableLED led;
    AddressableLEDBuffer ledbuffer;
    public LEDS(int length) {
        led = new AddressableLED(0);
        ledbuffer = new AddressableLEDBuffer(length);
        led.setLength(ledbuffer.getLength());

        if (DriverStation.isDisabled()) {
            for (int i = 0; i < ledbuffer.getLength(); ++i) {
                ledbuffer.setLED(i, Color.kDarkRed);
            }
        }
        if (DriverStation.isEnabled()) {
            for (int i = 0; i < ledbuffer.getLength(); ++i) {
                ledbuffer.setLED(i, Color.kGreen);
            }
        }

        led.setData(ledbuffer);
        led.start();
    }
}
