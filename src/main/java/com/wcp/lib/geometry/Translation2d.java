// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;


/** Add your docs here. */
public class Translation2d extends edu.wpi.first.math.geometry.Translation2d{
    protected edu.wpi.first.math.geometry.Translation2d conversion = new edu.wpi.first.math.geometry.Translation2d();
    protected double m_x;
    protected double m_y;
    public Translation2d() {
        this(0,0);
    }
    public Translation2d(double x, double y) {
        m_x = x;
        m_y = y;
    }
        public Translation2d(double[] pose) {
        m_x = pose[0];
        m_y = pose[1];
    }
    public Translation2d(final Translation2d otherVec) {
        m_x = otherVec.m_x;
        m_y = otherVec.m_y;
    }
    public Translation2d(final edu.wpi.first.math.geometry.Translation2d otherVec) {
        m_x = otherVec.getX();
        m_y = otherVec.getY();
    }
    public Translation2d minus(Translation2d other) {
        return new Translation2d(m_x - other.m_x, m_y - other.m_y);
      }
    
    public Rotation2d getAngle() {
        return new Rotation2d(m_x, m_y);
      }

    public static Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.m_cos * magnitude, direction.m_sin * magnitude);
    }

    public double norm() {
        return Math.hypot(m_x, m_y);
    }

    @Override
    public double getX() {
        return m_x;
    }
    @Override
    public double getY() {
        return m_y;
    }
    
  
    public Rotation2d angleToOther(Translation2d other){
        double deltax = m_x - other.m_x;
        double deltay = m_y = other.m_y;
        return new Translation2d(deltax,deltay).getAngle();
    }
    
    public static edu.wpi.first.math.geometry.Translation2d toWPI(double x, double y){
        return new edu.wpi.first.math.geometry.Translation2d(x,y);
    }

    public edu.wpi.first.math.geometry.Translation2d toWPI(){
        return new edu.wpi.first.math.geometry.Translation2d(m_x,m_y);
    }

    public Translation2d translateBy(final Translation2d other) {
        return new Translation2d(m_x + other.m_x, m_y + other.m_y);
    }
    
    public Translation2d rotateBy(final Rotation2d rotation) {
        return new Translation2d(m_x * rotation.cos() - m_y * rotation.sin(), m_x * rotation.sin() + m_y * rotation.cos());
    }

    public Rotation2d direction() {
        return new Rotation2d(m_x, m_y, true);
    }

    public Translation2d inverse() {
        return new Translation2d(-m_x, -m_y);
    }
    
    public Translation2d scale(double scaleFactor) {
        return new Translation2d(m_x * scaleFactor, m_y * scaleFactor);
    }

    public double distance(final Translation2d other) {
        return this.translateBy(other.inverse()).norm();
    }
    public double[] getArray(){
        double[] arrray ={m_x,m_y};
        return arrray;
    }
}
