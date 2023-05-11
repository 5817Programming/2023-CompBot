// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;

/** Add your docs here. */
public class Pose2dd {

    protected Translation2dd mTranslation;
    protected Rotation2dd mRotation;
    public Pose2dd() {
        mTranslation = new Translation2dd();
        mRotation = new Rotation2dd();
    }
    public Pose2dd(Translation2dd translation, Rotation2dd rotation) {
        mTranslation = translation;
        mRotation = rotation;
    }

    public static Pose2dd fromTranslation(final Translation2dd translation) {
        return new Pose2dd(translation, new Rotation2dd());
    }
    public static Pose2dd fromRotaiton(final Rotation2dd rotation) {
        return new Pose2dd(new Translation2dd(), rotation);
    }
        /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2dd inverse() {
        Rotation2dd rotation_inverted = mRotation.inverse();
        return new Pose2dd(mTranslation.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }
    
    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
        public Pose2dd transformBy(final Pose2dd other) {
        return new Pose2dd(mTranslation.translateBy(other.mTranslation.rotateBy(mRotation)),
                mRotation.rotateBy(other.mRotation));
    }
    public Pose2dd scale(double scaleFactor){
        return new Pose2dd(new Translation2dd(mTranslation.mX*scaleFactor,mTranslation.mY*scaleFactor),mRotation);
    }

    public Rotation2dd getRotation() {
        return this.mRotation;
    }

    public Translation2dd getTranslation() {
        return this.mTranslation;
    }

}
