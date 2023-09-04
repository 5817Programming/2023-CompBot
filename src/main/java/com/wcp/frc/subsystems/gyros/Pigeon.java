// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.gyros;

import com.ctre.phoenix.sensors.Pigeon2;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;

/** 
 * This class is used as an extention of the original pidgeon class
 * we improved on their methods by offsetting the error on get pitch and by inverting getangle in order to better suit our robots needs
 */
public class Pigeon extends Gyro {
    private static Pigeon instance = null;

    public static Pigeon getInstance() {
        if (instance == null)
            instance = new Pigeon();
        return instance;
    }

    Pigeon2 pigeon;

    public Pigeon() {
        try {
            pigeon = new Pigeon2(Ports.PIGEON);
            // secondPigeon = new PigeonIMU(Ports.SECONDARY_PIGEON);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public double getPitch() {
        return pigeon.getRoll() + 1.7;
    }

    public void setAngle(double angle) {
        pigeon.setYaw(angle);
        // pigeon.setFusedHeading(-angle * 64.0, Constants.kCANTimeoutMs);
        pigeon.setAccumZAngle(angle * 64.0, Constants.kCANTimeoutMs);

    }

    @Override
    public double getAngle() {
        return -pigeon.getYaw();
    }
}
