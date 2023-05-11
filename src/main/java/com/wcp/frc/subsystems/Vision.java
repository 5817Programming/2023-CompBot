// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.lib.Conversions;
import com.wcp.lib.geometry.Pose2dd;
import com.wcp.lib.geometry.Rotation2dd;
import com.wcp.lib.geometry.Translation2dd;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  
  Swerve swerve = Swerve.getInstance();
  double[] empty = {0.0,0.0,0.0,0.0,0.0,0.0};

  public double height;
  public static Vision instance = null;
  public int setPoint;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");//gets limelight data
  NetworkTableEntry tx = table.getEntry("tx");//gets specific data
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  NetworkTableEntry botpose = table.getEntry("botpose");

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
    Swerve swerve = Swerve.getInstance();
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
  
  /*
   * public void setfinished(boolean isFinishedd){
   * 
   * isFinished = isFinishedd;
   * }
   */

  public double getYaw() {
    yaw = tx.getDouble(0.0);
    return yaw;
  }
  public Pose2dd getPose(){
    double[] poseList = botpose.getDoubleArray(empty);
    Logger.getInstance().recordOutput("LimePose", new Pose2d(new Translation2d(poseList[0],poseList[1]), new Rotation2d()));
    return new Pose2dd(new Translation2dd(poseList[0],poseList[1]), new Rotation2dd());

  }
  public Pose2dd getWeightedPose(){
    double odometryWheight = (getDistance()- Constants.VisionConstants.lowerThreshold)/( Constants.VisionConstants.upperThreshold- Constants.VisionConstants.lowerThreshold);
    Logger.getInstance().recordOutput("wheghtedPose",Util.Poseconvert2ddto2d( getPose().scale(odometryWheight).transformBy(swerve.getPose().scale((1-odometryWheight)))));
    return getPose().scale(odometryWheight).transformBy(swerve.getPose().scale((1-odometryWheight)));
    }

  public boolean hasTarget() {//returns in binary so we convert to boolean 
    double v = tv.getDouble(0.0);
    if (v == 0.0f) {
      return false;
    } else {
      return true;
    }
  }
  public void setIndex(double pipelineIndex, int _setPoint){//sets pipline index
    table.getEntry("pipeline").setNumber(pipelineIndex);   
    this.setPoint = _setPoint;
     height = Constants.VisionConstants.HEIGHTS.get(setPoint);
     Logger.getInstance().recordOutput("heigh", height);   
     



  }

  public double getHeight(){
    if(swerve.getPose().getTranslation().x()>8.29){

      height = Constants.VisionConstants.HEIGHTS.get(2);
      
    }else{
     height = Constants.VisionConstants.HEIGHTS.get(setPoint);
    }
    return height;
  }

  public double getDistance(){//gets distance to target
    double distanceFromLimelightToGoalInches = (getHeight() - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }
  public void   setPipeline(Integer pipeline) {
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
  public double getDistanceToGroundObject(){//gets distance to target
    double distanceFromLimelightToGoalInches = (0 - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + getY()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }

  @Override
  public void periodic() {
    range = getDistance();
    Logger.getInstance().recordOutput("range", range);

  }

}
