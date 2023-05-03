// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.wcp.frc.subsystems.AutoBase;
import com.wcp.frc.subsystems.Odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class toHuman extends AutoBase{
    public toHuman (Odometry Odemetry){
        super(Odemetry);
        PathPlannerTrajectory toHuman;
        double currentvel = Odemetry.getAverageVel();
        if (Odemetry.getOdometry().getX()<2.4&&Odemetry.getOdometry().getY()<2.69){
            double heading  = Math.asin(Odemetry.getOdometry().getY()/(Math.hypot((Odemetry.getOdometry().getX()-2.43) ,(Odemetry.getOdometry().getY()-1))));

        toHuman = PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            new PathPoint(Odemetry.getOdometry().getTranslation(),Rotation2d.fromDegrees(-90), Odemetry.getOdometry().getRotation(), currentvel),
            new PathPoint(new Translation2d(2.43, 1), Rotation2d.fromDegrees(1.71),Odemetry.getOdometry().getRotation()),
            new PathPoint(new Translation2d(5.26, 1), Rotation2d.fromDegrees(1.71),Odemetry.getOdometry().getRotation()),
            new PathPoint(new Translation2d(12.78, 7.37), Rotation2d.fromDegrees(1.71), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(15.76, 7.37), Rotation2d.fromDegrees(175.36), Rotation2d.fromDegrees(0)));
        } else if (Odemetry.getOdometry().getX()<2.4&&Odemetry.getOdometry().getY()>2.69){
            double heading  = Math.asin(Odemetry.getOdometry().getY()/(Math.hypot((Odemetry.getOdometry().getX()-2.43) ,(Odemetry.getOdometry().getY()-4.57))));

            toHuman = PathPlanner.generatePath(
            
            new PathConstraints(4, 3), 
            new PathPoint(Odemetry.getOdometry().getTranslation(),Rotation2d.fromDegrees(90), Odemetry.getOdometry().getRotation(), currentvel),
            new PathPoint(new Translation2d(2.43, 4.57), Rotation2d.fromDegrees(1.71),Odemetry.getOdometry().getRotation()),
            new PathPoint(new Translation2d(5.26, 4.57), Rotation2d.fromDegrees(1.71),Odemetry.getOdometry().getRotation()),
            new PathPoint(new Translation2d(12.78, 7.37), Rotation2d.fromDegrees(1.71), Rotation2d.fromDegrees(0)),
            new PathPoint(new Translation2d(15.76, 7.37), Rotation2d.fromDegrees(175.36), Rotation2d.fromDegrees(0)));

        }
        else if(Odemetry.getOdometry().getX()<5.26&&Odemetry.getOdometry().getY()<2.69){
            double heading  = Math.asin(Odemetry.getOdometry().getY()/(Math.hypot((Odemetry.getOdometry().getX()-5.26) ,(Odemetry.getOdometry().getY()-1))));

            toHuman = PathPlanner.generatePath(
                new PathConstraints(4, 3), 
                new PathPoint(Odemetry.getOdometry().getTranslation(),Rotation2d.fromDegrees(0), Odemetry.getOdometry().getRotation(), currentvel),
                new PathPoint(new Translation2d(5.26, 1), Rotation2d.fromDegrees(1.71),Odemetry.getOdometry().getRotation()),
                new PathPoint(new Translation2d(12.78, 7.37), Rotation2d.fromDegrees(1.71), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(15.76, 7.37), Rotation2d.fromDegrees(175.36), Rotation2d.fromDegrees(0)));   
        } else if(Odemetry.getOdometry().getX()<5.26&&Odemetry.getOdometry().getY()>2.69){
            double heading  = Math.asin(Odemetry.getOdometry().getY()/(Math.hypot((Odemetry.getOdometry().getX()-5.26) ,(Odemetry.getOdometry().getY()-1.71))));

            toHuman = PathPlanner.generatePath(
                new PathConstraints(4, 3), 
                new PathPoint(Odemetry.getOdometry().getTranslation(),Rotation2d.fromDegrees(0), Odemetry.getOdometry().getRotation(), currentvel),
                new PathPoint(new Translation2d(5.26, 4.57), Rotation2d.fromDegrees(1.71),Odemetry.getOdometry().getRotation()),
                new PathPoint(new Translation2d(12.78, 7.37), Rotation2d.fromDegrees(1.71), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(15.76, 7.37), Rotation2d.fromDegrees(175.36), Rotation2d.fromDegrees(0)));   


        }else{
            toHuman = PathPlanner.generatePath(
                new PathConstraints(4, 3), 
                new PathPoint(Odemetry.getOdometry().getTranslation(),Rotation2d.fromDegrees(0), Odemetry.getOdometry().getRotation(), currentvel),
                new PathPoint(new Translation2d(12.78, 7.37), Rotation2d.fromDegrees(1.71), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(15.76, 7.37), Rotation2d.fromDegrees(175.36), Rotation2d.fromDegrees(0)));
        };
       

        PPSwerveControllerCommand followPath = baseSwerveCommand(toHuman, true);
        Logger.getInstance().recordOutput("trajectory",toHuman);
        addRequirements(Odemetry);
        addCommands(followPath);









    }

}
