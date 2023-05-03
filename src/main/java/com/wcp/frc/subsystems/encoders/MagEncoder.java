// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.encoders;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

/** Add your docs here. */
public class MagEncoder extends Encoder {
    DutyCycle encoder;
    public MagEncoder(int port) {
        encoder = new DutyCycle(new DigitalInput(port));
    }
    @Override
    public double getOutput() {
        return encoder.getOutput();
    }
    @Override
    public boolean isConnected() {
        return encoder.getFrequency() != 0;
    }
}
