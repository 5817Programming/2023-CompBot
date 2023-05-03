// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.encoders;

import com.ctre.phoenix.sensors.CANCoder;

/** Add your docs here. */
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
