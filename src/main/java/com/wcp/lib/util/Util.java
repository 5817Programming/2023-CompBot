// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.util;

import com.wcp.lib.geometry.Pose2dd;
import com.wcp.lib.geometry.Rotation2dd;
import com.wcp.lib.geometry.Translation2dd;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Util {
    public static final double kEpsilon = 1e-12;

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle){
    	double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0){
        	lowerBound = scopeReference - lowerOffset;
        	upperBound = scopeReference + (360 - lowerOffset);
        }else{
        	upperBound = scopeReference - lowerOffset; 
        	lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
        	newAngle += 360; 
        }
        while(newAngle > upperBound){
        	newAngle -= 360; 
        }
        if(newAngle - scopeReference > 180){
        	newAngle -= 360;
        }else if(newAngle - scopeReference < -180){
        	newAngle += 360;
        }
        return newAngle;
    }

    public static double deadband(double deadbandValue, double value) {
        if(deadbandValue > value)
            return deadbandValue;
        else
            return value;
    }
    public static boolean shouldReverse(Rotation2dd goalAngle, Rotation2dd currentAngle) {
        double angleDifferene = Math.abs(goalAngle.distance(currentAngle));
        double reversedAngleDifference = Math.abs(goalAngle.distance(currentAngle.rotateBy(Rotation2dd.fromDegrees(180.0))));
        return reversedAngleDifference < angleDifferene;
    }
    public static Translation2dd Translationconvert2dto2dd( Translation2d translation2d){
        return new Translation2dd(translation2d.getX(),translation2d.getY());
    }
    public static Translation2d Translationconvert2ddto2d( Translation2dd translation2dd){
        return new Translation2d(translation2dd.x(),translation2dd.y());
    } 
    public static Rotation2dd Rotationconvert2dto2dd( Rotation2d rotation2d){
        return Rotation2dd.fromDegrees(rotation2d.getDegrees());
    }
    public static Rotation2d Rotationconvert2ddto2d( Rotation2dd rotation2dd){
        return Rotation2d.fromDegrees(rotation2dd.getDegrees());
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
    
    public static Pose2d Poseconvert2ddto2d(Pose2dd pose2dd){
        return new Pose2d(Translationconvert2ddto2d(pose2dd.getTranslation()), Rotationconvert2ddto2d(pose2dd.getRotation()));
    }
    public static Pose2dd Poseconvert2dto2dd(Pose2d pose2d){
        return new Pose2dd(Translationconvert2dto2dd(pose2d.getTranslation()), Rotationconvert2dto2dd(pose2d.getRotation()));
    }
   
   
   
    
}
