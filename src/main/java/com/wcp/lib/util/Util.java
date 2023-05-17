// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.util;

import com.wcp.lib.geometry.Rotation2d;


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
    public static boolean shouldReverse(Rotation2d goalAngle, Rotation2d currentAngle) {
        double angleDifferene = Math.abs(goalAngle.distance(currentAngle));
        double reversedAngleDifference = Math.abs(goalAngle.distance(currentAngle.rotateBy(Rotation2d.fromDegrees(180.0))));
        return reversedAngleDifference < angleDifferene;
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
    
   
   
   
    
}
