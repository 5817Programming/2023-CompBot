// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;

import static com.wcp.lib.util.Util.kEpsilon;

/** Add your docs here. */
public class Rotation2d extends edu.wpi.first.math.geometry.Rotation2d{
    protected double m_cos;
    protected double m_sin;

    public Rotation2d() {
        this(1,0, false);
    }
    public Rotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
          m_sin = y / magnitude;
          m_cos = x / magnitude;
        } else {
          m_sin = 0.0;
          m_cos = 1.0;
        }
    }
    
    public Rotation2d(double x, double y, boolean normalize) {
        if(normalize) {
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                m_cos = x / magnitude;
                m_sin = y / magnitude;
            } else {
                m_cos = 1;
                m_sin = 0;
            }
        } else {
            m_cos = x;
            m_sin = y;
        }
    }
    
    public Rotation2d(final Rotation2d otherRotation) {
        m_cos = otherRotation.m_cos;
        m_sin = otherRotation.m_sin;
    }
    
    public Rotation2d(double thetaDegrees) {
        m_cos = Math.cos(Math.toRadians(thetaDegrees));
        m_sin = Math.sin(Math.toRadians(thetaDegrees));
    }
    
    public static Rotation2d fromDegrees(double angle) {
        return new Rotation2d(angle);
    }
    
    public double cos() {
        return m_cos;
    }
    public double sin() {
        return m_sin;
    }
    @Override
    public double getRadians() {
        return Math.atan2(m_sin, m_cos);
    }
    @Override
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }
    
    public Rotation2d rotateBy(final Rotation2d other) {
        return new Rotation2d(m_cos * other.m_cos - m_sin * other.m_sin, m_cos * other.m_sin + m_sin * other.m_cos, true);
    }
    public edu.wpi.first.math.geometry.Rotation2d toWPI(){
        return new edu.wpi.first.math.geometry.Rotation2d(m_cos,m_sin);
    }
    
    public double distance(Rotation2d other) {
        return this.rotateBy(other.inverse()).getRadians();
    }
    
    public Rotation2d inverse() {
        return new Rotation2d(m_cos, -m_sin, false);
    }
    
    public Translation2d toVector2d() {
        return new Translation2d(m_cos, m_sin);
    }
}
