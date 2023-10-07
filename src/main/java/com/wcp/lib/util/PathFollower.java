package com.wcp.lib.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;

import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class PathFollower extends SubsystemBase {
  private final Timer timer = new Timer();
  private final Timer waitTimer = new Timer();

  private PathPlannerTrajectory transformedTrajectory;
  private PathPlannerState startState;

  double desiredRotation = 0;
  double speed=1;
  Pose2d currentPose;
  boolean useEvents = false;
  boolean ran= false;
  boolean red= false;

  int EventIndex= 0;

  

  
  public PathFollower() {
      }
   
  public void startTimer(){
    this.timer.start();
  }
  public void setTrajectory (PathPlannerTrajectory trajectory){
    resetTimer();
    this.transformedTrajectory = trajectory;
    Logger.getInstance().recordOutput("Trajectory", this.transformedTrajectory);

    if(DriverStation.getAlliance()==Alliance.Blue){
      red=false;
    }
    else{
      red=true;
    }

  }

 

  

  public Pose2d getDesiredPose2d(
    boolean useAllianceColor, double speed, Pose2d currentPose2d) {

    this.currentPose = currentPose2d;
    this.timer.start();
    double currentTime = this.timer.get()*.5;
    this.speed = speed;

    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);


double desiredX = desiredState.poseMeters.getTranslation().getX();
double desiredY = desiredState.poseMeters.getTranslation().getY();
double desiredRotation =  desiredState.holonomicRotation.getDegrees();
this.desiredRotation = desiredRotation;
      Logger.getInstance().recordOutput("Current time", currentTime);
    Logger.getInstance().recordOutput("desiredPose", new Pose2d(new Translation2d(desiredX,desiredY), Rotation2d.fromDegrees(desiredRotation)));

    if(red&&useAllianceColor){
      return   new Pose2d(new Translation2d(desiredX+(2*Math.abs(8.25-desiredX)),desiredY), Rotation2d.fromDegrees(desiredRotation-180));
    }else{
      return   new Pose2d(new Translation2d(desiredX,desiredY), Rotation2d.fromDegrees(desiredRotation));
    }
    
  }
  
  public static PathFollower instance = null;

  public static PathFollower getInstance() {// if doesnt have an instance of swerve will make a new one
    if (instance == null)
        instance = new PathFollower();
    return instance;
}
public double getrotation(){
return desiredRotation;
}
public Pose2d getStart(){
  
  startState = ((PathPlannerState) transformedTrajectory.sample(.0000000000000000000000000000000000000001));
  if(red){
    return new Pose2d(startState.poseMeters.getX()+(2*Math.abs(8.25-startState.poseMeters.getX())),-(startState.poseMeters.getY()),Rotation2d.fromDegrees(startState.holonomicRotation.getDegrees()));
  }
  return new Pose2d(startState.poseMeters.getX(),startState.poseMeters.getY(),Rotation2d.fromDegrees(startState.holonomicRotation.getDegrees()));
}
public double getStartRotation(){
  if (red){
    return startState.holonomicRotation.getDegrees()+180;
  }
  return startState.holonomicRotation.getDegrees();
   
}


  public void resetTimer(){
    timer.reset();
    timer.stop();
  }
 

  public boolean isFinished() {
    double extraSeconds =0;

    if (this.timer.hasElapsed((transformedTrajectory.getTotalTimeSeconds()+extraSeconds/speed)+2)){
      EventIndex = 0;
      useEvents = false;
    }
    return this.timer.hasElapsed((transformedTrajectory.getTotalTimeSeconds()/.5)+2);
    
  }
 


  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a PPSwerveControllerCommand starts
   * @param logTargetPose Consumer that accepts a Pose2d representing the target pose while path
   *     following
   * @param logSetpoint Consumer that accepts a ChassisSpeeds object representing the setpoint
   *     speeds
   * @param logError BiConsumer that accepts a Translation2d and Rotation2d representing the error
   *     while path following
   */

}
