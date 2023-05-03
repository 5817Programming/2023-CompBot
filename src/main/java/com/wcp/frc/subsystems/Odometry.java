// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.HeadingController;
import com.wcp.lib.SwerveInverseKinematics;
import com.wcp.lib.geometry.Pose2dd;
import com.wcp.lib.geometry.Rotation2dd;
import com.wcp.lib.geometry.Translation2dd;
import com.wcp.lib.util.ScuffedPathPlanner;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class Odometry extends SubsystemBase {
    

    //making an instance for swerve
    public static Odometry instance = null;
    public static Odometry getInstance() {//if has no instance will make a new one
        if(instance == null)
            instance = new Odometry();
        return instance;
    }
    //initilizes position
    public double X=0;
    public double Y=0;
    public double rotation=0;
    //initilizes subsystems
    Gyro pigeon;
    Swerve swerve;
    Vision vision;
    SwerveDriveModule frontLeftModule, frontRightModule, rearLeftModule, rearRightModule; //makes swerve modules
    ScuffedPathPlanner scuffedPathPlanner;
    SwerveModulePosition frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition; //makes swerve modules
    SwerveModulePosition[] modulePositions;
    SwerveModulePosition[] modulePositionSIM;

    Pose2d position = new Pose2d();//resets pos
    Pose3d positionsim = new Pose3d(5,5,0,new Rotation3d(180,270,0));//resets pos for sim
    String sector;
    boolean trajectoryStarted = false;
    //dead zones

//initilization
    SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
    HeadingController headingController = new HeadingController();
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.modulePositions);
    List<SwerveDriveModule> modules;
    SwerveDriveOdometry odometry;
    double simHeading= 0;

    

    public Odometry() {
        //gets instances
        pigeon= Pigeon.getInstance();
      swerve=Swerve.getInstance();
       this.frontRightModule = swerve.frontRightModule;//gets modules
       this.frontLeftModule = swerve.frontLeftModule;
       this.rearLeftModule = swerve.rearLeftModule;
       this.rearRightModule = swerve.rearRightModule;

                modules = Arrays.asList(frontRightModule, rearRightModule, frontLeftModule, rearLeftModule);

        modulePositions = getModulePositions();
        scuffedPathPlanner = ScuffedPathPlanner.getInstance();
        
            
        //makes modules
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(pigeon.getAngle()),modulePositions);


    }
 
    public void setModuleStates(SwerveModuleState[] moduleState){
     swerve.setModuleStates(moduleState);
       

    }
    public double getAverageVel(){
        return swerve.getAverageVel();
    }
    public void resetEncoders(){
       swerve.resetEncoders();

    }
    public BooleanSupplier TrajectoryisFinished (){
        return new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                if (swerve.isTrajectoryFollowed()){
                    trajectoryStarted = false;
                }
                // TODO Auto-generated method stub
                return swerve.isTrajectoryFollowed();
            }
    };
};
    public Pose2d getOdometry (){
        return position;
    }
    public Pose2dd getPose2dd() {
        return new Pose2dd(new Translation2dd(Units.metersToInches(getOdometry().getTranslation().getX()), Units.metersToInches(getOdometry().getTranslation().getY())),
                    new Rotation2dd(getOdometry().getRotation().getDegrees()));
    }
    public Pose2dd getPose2ddMeters() {
        return Util.Poseconvert2dto2dd(position);
    }
    public void resetOdometry (Pose2d newpose){
       swerve.resetPose();
  }


public void setTrajectory (PathPlannerTrajectory trajectory,List<Translation2d> eventTimings, List<Command> events, List<Double> waitTiming) {
    scuffedPathPlanner.setTrajectory(trajectory);
    scuffedPathPlanner.setEventTimings(eventTimings);
    scuffedPathPlanner.setEvents(events);
    scuffedPathPlanner.setWaitTimings(waitTiming);
    resetOdometry(scuffedPathPlanner.getStart()); 
    swerve.resetGryo(scuffedPathPlanner.getStartRotation());
    Logger.getInstance().recordOutput("startRotation", scuffedPathPlanner.getStartRotation());
    swerve.setTrajectory(trajectory);
    resetTimer();

}

public void resetTimer (){
    ScuffedPathPlanner.getInstance().resetTimer();
}


    public SwerveModulePosition[] getModulePositions() {//makes a list of the module positions
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < modules.size(); i++) {
            positions[i]= modules.get(i).getPosition();
            
        }
        return positions;
    }


    public SwerveModuleState[] getModuleState() {//makes a list of the module states
        SwerveModuleState[] positions = new SwerveModuleState[4];
        for(int i = 0; i < modules.size(); i++) {
            positions[i]= modules.get(i).getState();
            
        }
        return positions;
    }
    
    private void logModuleStates(String key, SwerveModuleState[] states) {//gives module data to advantage scope
        List<Double> dataArray = new ArrayList<Double>();
        for (int i = 0; i < 4; i++) {//makes list
          dataArray.add(states[i].angle.getRadians());
          dataArray.add(states[i].speedMetersPerSecond);
        }
        Logger.getInstance().recordOutput(key,
            dataArray.stream().mapToDouble(Double::doubleValue).toArray());//sends list
      }

      
      private void logPos(Pose2d pos) {
       Logger.getInstance().recordOutput("Odometry", pos);//gives pos to advantage scope
      }
      public double getX(){

        return position.getX();
      }
 
      public void sendInput(double x , double y, double r){
        swerve.sendInput(x, y, r);
      }
  
    public void update() {
        swerve.updatePose(Timer.getFPGATimestamp());
        position = Util.Poseconvert2ddto2d(swerve.pose);
       // X = position.getX();
       // Y = position.getY();
        //rotation = position.getRotation().getDegrees();
        logModuleStates("States", getModuleState());
        //sends states to advantage scope
        logPos(position);
        Logger.getInstance().recordOutput("X", X);
        //  if(vision.hasTarget()){
        //     aprilreset();
        //  }
        
    }

  }


   