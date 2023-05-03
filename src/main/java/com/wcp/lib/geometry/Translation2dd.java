// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;


/** Add your docs here. */
public class Translation2dd {

    protected double mX;
    protected double mY;
    public Translation2dd() {
        this(0,0);
    }
    public Translation2dd(double x, double y) {
        mX = x;
        mY = y;
    }
    public Translation2dd(final Translation2dd otherVec) {
        mX = otherVec.mX;
        mY = otherVec.mY;
    }

    public static Translation2dd fromPolar(Rotation2dd direction, double magnitude) {
        return new Translation2dd(direction.mCos * magnitude, direction.mSin * magnitude);
    }

    public double norm() {
        return Math.hypot(mX, mY);
    }

    public double x() {
        return mX;
    }
    public double y() {
        return mY;
    }

    public Translation2dd translateBy(final Translation2dd other) {
        return new Translation2dd(mX + other.mX, mY + other.mY);
    }
    public Translation2dd minus(final Translation2dd other) {
        return new Translation2dd(mX - other.mX, mY - other.mY);
    }
    
    
    
    public Translation2dd rotateBy(final Rotation2dd rotation) {
        return new Translation2dd(mX * rotation.cos() - mY * rotation.sin(), mX * rotation.sin() + mY * rotation.cos());
    }

    public Rotation2dd direction() {
        return new Rotation2dd(mX, mY, true);
    }

    public Translation2dd inverse() {
        return new Translation2dd(-mX, -mY);
    }
    
    public Translation2dd scale(double scaleFactor) {
        return new Translation2dd(mX * scaleFactor, mY * scaleFactor);
    }

    public double distance(final Translation2dd other) {
        return this.translateBy(other.inverse()).norm();
    }
}
