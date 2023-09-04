// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.encoders;

import com.ctre.phoenix.sensors.CANCoder;

/** 
 * This class inherits from "Encoder" see "Encoder.java"
 * We made this class so we can customize the behavior of the code that they wrote for us
 */
public class CANEncoder extends Encoder{
    CANCoder encoder;
    public CANEncoder(int id) {
        encoder = new CANCoder(id);
    }
    @Override
    public double getOutput() {
        return encoder.getAbsolutePosition();
    }
    @Override
    public boolean isConnected() {
        return encoder.getBusVoltage() != 0;
    }
    
}
