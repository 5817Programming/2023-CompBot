// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.gyros;

/** Add your docs here. */
public abstract class Gyro {
    /**
     * @param angle Sets the angle of the gyro angle(used for resetting the robot's heading)
     */
    public abstract void setAngle(double angle);
    /**
     * @return Returns the gyros angle, in degrees
     */
    public abstract double getAngle();
    public abstract double getPitch();

}
