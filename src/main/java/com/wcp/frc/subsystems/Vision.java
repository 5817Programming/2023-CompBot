// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.lang.module.ResolutionException;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.frc.Constants.VisionConstants;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.lib.Conversions;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends Subsystem {
  double lastY;
  double lastX;
  Swerve swerve = Swerve.getInstance();
  double[] empty = {5.0,5.0,0.0,0.0,0.0,0.0};

  public double height;
  Pose2d lastPose;
  public static Vision instance = null;
  public int setPoint;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");//gets limelight data
  NetworkTableEntry tx = table.getEntry("tx");//gets specific data
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  // NetworkTableEntry botpose = table.getEntry("botpose");
  NetworkTableEntry botpose = table.getEntry("botpose");
  DoubleArraySubscriber posesub = table.getDoubleArrayTopic("botpose").subscribe(empty);


  double x;
  double y;
  double area;
  double range;
  double yaw;
  

  public static Vision getInstance() {
    if (instance == null)
      instance = new Vision();
    return instance;
  }

  /** Creates a new Vision. */
  public Vision() {
    // read values periodically
    // post to smart dashboard periodically

  }

  public double getX() {
    x = tx.getDouble(0.0);//gets limelight x
    return x;
  }

  public double getY() {
    double y = ty.getDouble(0.0);//gets limelight y
    return y;
  }

  public double getArea() {
    double area = ta.getDouble(0.0);//gets limelight area
    return area;
  }


  public double getYaw() {
    yaw = tx.getDouble(0.0);
    return yaw;
  }
  public Pose2d getPose(){
    double[] result = posesub.get();
    
    Logger.getInstance().recordOutput("LimePose", new Pose2d(new Translation2d(result[0]+8.2296,result[1]+3.9624), new Rotation2d()));
    lastPose = new Pose2d(new Translation2d(result[0]+8.2296,result[1]+3.9624), new Rotation2d());
    return new Pose2d(new Translation2d(result[0]+8.2296,result[1]+3.9624), new Rotation2d());

  }
  public Pose2d getWeightedPose(Pose2d odomotryPose){
    if(getPose().getTranslation().distance(lastPose.getTranslation()) < .1){
      return getPose();
    }else{
      return odomotryPose;
    }
    
  }

  public boolean hasTarget() {//returns in binary so we convert to boolean 
    double v = tv.getDouble(0.0);
    if (v == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  // public boolean hasTarget() {//returns in binary so we convert to boolean 
  //   double currentX = getX();
  //   if(lastX != currentX){
  //     lastX = currentX;
  //     return true; 
  //   }
  //   else{
  //     lastX = currentX;
  //     return false;
  //   }
  // }
  
  public void setIndex(double pipelineIndex, int _setPoint){//sets pipline index
    table.getEntry("pipeline").setNumber(pipelineIndex);   
    this.setPoint = _setPoint;
     height = Constants.VisionConstants.HEIGHTS.get(setPoint);
     



  }

  public double getHeight(){
    if(swerve.getPose().getTranslation().getX()>8.29){

      height = Constants.VisionConstants.HEIGHTS.get(2);
      
    }else{
     height = Constants.VisionConstants.HEIGHTS.get(setPoint);
    }
    return height;
  }

  public double getDistance(){//gets distance to target
    double distanceFromLimelightToGoalInches = (Constants.VisionConstants.APRIL_HEIGHT_INCHES - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }
  public double getDistanceObject(){//gets distance to target
    double distanceFromLimelightToGoalInches = (-Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }
  public void setPipeline(Integer pipeline) {
    if(pipeline<0){
        pipeline = 0;
        throw new IllegalArgumentException("Pipeline can not be less than zero");
    }else if(pipeline>9){
        pipeline = 9;
        throw new IllegalArgumentException("Pipeline can not be greater than nine");
    }
    table.getEntry("pipeline").setValue(pipeline);
  }
  public void updatePipe(Boolean p){
    if (p){
      setIndex(Constants.VisionConstants.APRIL_PIPLINE, 0);
    }else{
      setIndex(Constants.VisionConstants.LOW_RETRO_PIPLINE, 1);

    }

  }

  public Request pipleLineRequest(int pipeline){
    return new Request () {
      public void act() {
          setPipeline(pipeline);
      }
    };
  }
  public double getDistanceToGroundObject(){//gets distance to target
    double distanceFromLimelightToGoalInches = (0 - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }
  @Override
  public void update(){
    Logger.getInstance().recordOutput("Xpos", getX());   
    
    getPose();
    hasTarget();
  }

  @Override
  public void outputTelemetry() {
    Logger.getInstance().recordOutput("Distance", getDistance());
    Logger.getInstance().recordOutput("hasTarget", hasTarget());
    Logger.getInstance().recordOutput("tx", tx.getDouble(0.0));
    Logger.getInstance().recordOutput("ty", ty.getDouble(0.0));
    Logger.getInstance().recordOutput("ta", ta.getDouble(0.0));
    

  }
  

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    
  }



}
