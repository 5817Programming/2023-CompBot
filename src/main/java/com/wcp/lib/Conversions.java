// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib;

/** Add your docs here. */
public class Conversions {
public static double CANCcoderToDegrees(double positionCounts, double gearRatio){
    return positionCounts*(360/(gearRatio* 4096));


}

public static double degreesToFalcom(double degrees, double gearRatio){
    return degrees/(360/(gearRatio*2048));
}

public static double RPMToFalcon(double RPM, double gearRation){
    double motorRPM = RPM*gearRation;
    double senseorCounts = motorRPM*(2048/600);
    return senseorCounts;

}
public static double falconToRPM(double velocitycounts, double gearRatio){
double motorRPM = velocitycounts*(600/2048);
double mechRPM = motorRPM/gearRatio;
return mechRPM;
}
    
    public static double falconToMPS(double veloctycounts, double circumference, double gearRatio){
    double wheelRPM = falconToRPM(veloctycounts, gearRatio);
    double wheelMPS = (wheelRPM*circumference)/60;
    return wheelMPS;

}
public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
    double wheelRPM = ((velocity*60)/circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;

}

public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
    return positionCounts*(circumference/(gearRatio*2048));
}

public static double MetersToFalcon(double meters, double circumference, double gearRatio){
return meters/(circumference/(gearRatio*2048));

}
public static double falconToDegrees(double positionCounts, double gearRatio) {
    return positionCounts * (360.0 / (gearRatio * 2048.0));
}
public static double degreesToFalcon(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 2048.0));
}
}
