// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;

import static com.wcp.lib.util.Util.kEpsilon;
/** Add your docs here. */
public class Rotation2dd {
    
    protected double mCos;
    protected double mSin;

    public Rotation2dd() {
        this(1,0, false);
    }
    public Rotation2dd(double x, double y, boolean normalize) {
        if(normalize) {
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                mCos = x / magnitude;
                mSin = y / magnitude;
            } else {
                mCos = 1;
                mSin = 0;
            }
        } else {
            mCos = x;
            mSin = y;
        }
    }
    public Rotation2dd(final Rotation2dd otherRotation) {
        mCos = otherRotation.mCos;
        mSin = otherRotation.mSin;
    }
    public Rotation2dd(double thetaDegrees) {
        mCos = Math.cos(Math.toRadians(thetaDegrees));
        mSin = Math.sin(Math.toRadians(thetaDegrees));
    }

    public static Rotation2dd fromDegrees(double angle) {
        return new Rotation2dd(angle);
    }

    public double cos() {
        return mCos;
    }
    public double sin() {
        return mSin;
    }

    public double getRadians() {
        return Math.atan2(mSin, mCos);
    }
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }
    public Rotation2dd rotateBy(final Rotation2dd other) {
        return new Rotation2dd(mCos * other.mCos - mSin * other.mSin, mCos * other.mSin + mSin * other.mCos, true);
    }
    public double distance(Rotation2dd other) {
        return this.rotateBy(other.inverse()).getRadians();
    }

    public Rotation2dd inverse() {
        return new Rotation2dd(mCos, -mSin, false);
    }

    public Translation2dd toVector2d() {
        return new Translation2dd(mCos, mSin);
    }
}
