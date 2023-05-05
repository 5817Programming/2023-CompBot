package com.wcp.lib.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import com.wcp.lib.geometry.Pose2dd;
import com.wcp.lib.geometry.Rotation2dd;
import com.wcp.lib.geometry.Translation2dd;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class ScuffedPathPlanner extends SubsystemBase {
  private final Timer timer = new Timer();
  private final Timer waitTimer = new Timer();

  private PathPlannerTrajectory transformedTrajectory;
  private PathPlannerState currentState;
  private PathPlannerState startState;

  double desiredRotation = 0;
  double speed=1;
  Pose2d currentPose;
  boolean useEvents = false;
  boolean ran= false;
  boolean red= false;
  
  List<Translation2d> eventTimings;
  List<Double> waitTimings;
  List <Command> events;
  int EventIndex= 0;

  

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError =
      ScuffedPathPlanner::defaultLogError;

  
  public ScuffedPathPlanner() {
      }
   
  public void startTimer(){
    this.timer.start();
  }
  public void setTrajectory (PathPlannerTrajectory trajectory){
    this.transformedTrajectory = trajectory;
    if(DriverStation.getAlliance()==Alliance.Blue){
      red=false;
    }
    else{
      red=true;
    }

  }
  public void setEventTimings(List<Translation2d> eventTimings){
    this.eventTimings = eventTimings;
    useEvents = true;
  }
  public void setWaitTimings(List<Double> waitTimings){
    this.waitTimings= waitTimings;
  }
  public void setEvents (List<Command> events){
    this.events = events;
    useEvents = true;
  }

  

  public Pose2dd getDesiredPose2dd(
  boolean useAllianceColor, double speed, Pose2d currenPose2d) {

    this.currentPose = currenPose2d;
    double currentTime = this.timer.get()*.5;
    this.speed = speed;
    if (eventTimings.size()>0){
      runEvents();
    }

    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);
    currentState = desiredState;


double desiredX = desiredState.poseMeters.getTranslation().getX();
double desiredY = desiredState.poseMeters.getTranslation().getY();
double desiredRotation =  desiredState.holonomicRotation.getDegrees();
this.desiredRotation = desiredRotation;
    Logger.getInstance().recordOutput("desiredPose", new Pose2d(new Translation2d(desiredX,desiredY), Rotation2d.fromDegrees(desiredRotation)));
    if(red){
      return   new Pose2dd(new Translation2dd(desiredX+(2*Math.abs(8.25-desiredX)),desiredY), Rotation2dd.fromDegrees(desiredRotation-180));
    }else{
      return   new Pose2dd(new Translation2dd(desiredX,desiredY), Rotation2dd.fromDegrees(desiredRotation));
    }
    
    
  }
  
  public static ScuffedPathPlanner instance = null;

  public static ScuffedPathPlanner getInstance() {// if doesnt have an instance of swerve will make a new one
    if (instance == null)
        instance = new ScuffedPathPlanner();
    return instance;
}
public double getrotation(){
return desiredRotation;
}
public Pose2d getStart(){
  
  startState = ((PathPlannerState) transformedTrajectory.sample(.0000000000000000000000000000000000000001));
  if(red){
    return new Pose2d(startState.poseMeters.getX()+(2*Math.abs(8.25-startState.poseMeters.getX())),-(startState.poseMeters.getY()),startState.holonomicRotation);
  }
  return startState.poseMeters;
}
public double getStartRotation(){
  if (red){
    return startState.holonomicRotation.getDegrees()+180;
  }
  return startState.holonomicRotation.getDegrees();
   
}


  public void resetTimer(){
    timer.reset();
  }
  public void runEvents(){
    Logger.getInstance().recordOutput("WaitTimer", waitTimer.get());
    Logger.getInstance().recordOutput("WaitTiming", waitTimings.get(EventIndex));
    Logger.getInstance().recordOutput("EventIndex", EventIndex);

    Logger.getInstance().recordOutput("currentPoseHypot", currentPose.getTranslation().getNorm());
    if (EventIndex<waitTimings.size()-1){
      if(red){
        if((Math.abs(currentPose.getTranslation().getY()-eventTimings.get(EventIndex).getY())<.1)&& Math.abs(currentPose.getTranslation().getX()-(eventTimings.get(EventIndex).getX()+(2*Math.abs(8.25-eventTimings.get(EventIndex).getX()))))<.05) {
          runEvent();
    }
      }else{
        if((Math.abs(currentPose.getTranslation().getY()-eventTimings.get(EventIndex).getY())<.1)&& Math.abs(currentPose.getTranslation().getX()-eventTimings.get(EventIndex).getX())<.06) {
          runEvent();
    }
      }
      }
  }
  void runEvent(){
    if(!ran){
      events.get(EventIndex).schedule();
    }
    ran= true;
    timer.stop();
    waitTimer.start();
    if (waitTimer.hasElapsed(waitTimings.get(EventIndex))){
      timer.start();
      waitTimer.reset();
      waitTimer.stop();
      EventIndex++;
      ran = false;
    }
  }
  

  public boolean isFinished() {
    double extraSeconds =0;
    for (int i=0; i< waitTimings.size();i++){
      extraSeconds += waitTimings.get(i);
    }
    if (this.timer.hasElapsed((transformedTrajectory.getTotalTimeSeconds()+extraSeconds/speed)+2)){
      EventIndex = 0;
      useEvents = false;
    }
    return this.timer.hasElapsed((transformedTrajectory.getTotalTimeSeconds()/.5)+2);
    
  }
 

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPSwerveControllerCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPSwerveControllerCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
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
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    ScuffedPathPlanner.logActiveTrajectory = logActiveTrajectory;
    ScuffedPathPlanner.logTargetPose = logTargetPose;
    ScuffedPathPlanner.logSetpoint = logSetpoint;
    ScuffedPathPlanner.logError = logError;
  }
}
