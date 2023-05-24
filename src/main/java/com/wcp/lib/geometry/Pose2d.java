// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;



/** Add your docs here. */
public class Pose2d extends edu.wpi.first.math.geometry.Pose2d {

    protected Translation2d m_translation;
    protected Rotation2d m_rotation;
    public Pose2d() {
        m_translation = new Translation2d();
        m_rotation = new Rotation2d();
    }
    public Pose2d(Translation2d translation, Rotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }
        public Pose2d(double x, double y, Rotation2d rotation) {
        m_translation = new Translation2d(x,y);
        m_rotation = rotation;
    }
            public  edu.wpi.first.math.geometry.Pose2d toWPI() {
        return new edu.wpi.first.math.geometry.Pose2d(m_translation.m_x, m_translation.m_y, m_rotation);
    }
    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }


    public static Pose2d fromRotaiton(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }
        public static edu.wpi.first.math.geometry.Pose2d toWPI(Translation2d translation, Rotation2d rotation) {
        return new Pose2d(translation, rotation);
    }
    public static edu.wpi.first.math.geometry.Pose2d toWPI(Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
        }

            /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2d inverse() {
        Rotation2d rotation_inverted = m_rotation.inverse();
        return new Pose2d(m_translation.inverse().rotateBy(rotation_inverted), rotation_inverted);

    }
    public Pose2d scale(double scaleFactor){
        return new Pose2d(new Translation2d(m_translation.m_x*scaleFactor,m_translation.m_x*scaleFactor),m_rotation);
    }
    
    @Override
    public Rotation2d getRotation() {
        return this.m_rotation;
    }
    @Override
    public double getX(){
        return m_translation.m_x;
    }
    @Override
    public double getY(){
        return m_translation.m_y;
    }
    
    
    @Override
    public Translation2d getTranslation() {
        return this.m_translation;
    }
        /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(m_translation.translateBy(other.m_translation.rotateBy(m_rotation)),
                m_rotation.rotateBy(other.m_rotation));
    }

}
