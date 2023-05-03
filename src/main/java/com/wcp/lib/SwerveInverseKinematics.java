// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib;

import java.util.ArrayList;
import java.util.List;

import com.wcp.frc.Constants;
import com.wcp.lib.geometry.Pose2dd;
import com.wcp.lib.geometry.Rotation2dd;
import com.wcp.lib.geometry.Translation2dd;

/** Add your docs here. */
public class SwerveInverseKinematics {

    public SwerveInverseKinematics() {
        updateModuleRotationVectors();
    }
    
    private final int kNumberOfModules = Constants.kModulePositions.size();
    private List<Translation2dd> moduleRotationVectors;

    
    public void updateModuleRotationVectors() {
        int numberOfModules = kNumberOfModules;
        double rotateVectorDirection = 360.0 / numberOfModules;
        List<Translation2dd> vectorList = new ArrayList<>(numberOfModules);
        for(int i = 0; i < numberOfModules; i++) {
            vectorList.add(Constants.kModulePositions.get(i).rotateBy(Rotation2dd.fromDegrees(rotateVectorDirection)));
        }
        moduleRotationVectors = vectorList;
    }
    /**
     * 
     * @param translationVector -A vector that that is usually represented by the translational stick on the controller
     * @param rotationalMagnitude -The amount of rotation of the swerve drive represented by the rotational stick on the controller
     * @param robotPosition -The Position of the robot, with respect to the Rotation
     * @param robotCentric -Should the determine it's heading based off of the robot. False = Field Centric(i.e, when you push forward, 
     *                      the robot will always move in the same direction, no matter it's heading)
     */
    public List<Translation2dd> updateDriveVectors(Translation2dd translationVector, double rotationalMagnitude, Pose2dd robotPosition, boolean robotCentric) {
        List<Translation2dd> driveVectors = new ArrayList<>(kNumberOfModules);
        if(!robotCentric)
            translationVector = translationVector.rotateBy(robotPosition.getRotation().inverse()); //Rotates by the translation vector by inverse rotation of the robot 
        for(int i = 0; i < kNumberOfModules; i++) {
            driveVectors.add(translationVector.translateBy(moduleRotationVectors.get(i).scale(rotationalMagnitude))); //Rotates the translation vector of each of the modules, by the rotation value
        }

        double maxMagnitude = 1.0;
        for (Translation2dd m : driveVectors) {
            double moduleVectorMagnitude = m.norm();
            if(moduleVectorMagnitude > maxMagnitude)
                maxMagnitude = moduleVectorMagnitude;
        }
        for (int i =0; i< kNumberOfModules; i++) {
            Translation2dd driveVector = driveVectors.get(i);
            driveVectors.set(i, driveVector.scale(1.0/maxMagnitude));
        }
        return driveVectors;
        
    }
}
