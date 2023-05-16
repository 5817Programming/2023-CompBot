// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.fasterxml.jackson.databind.jsontype.impl.AsExternalTypeDeserializer;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.Conversions;
import com.wcp.lib.HeadingController;
import com.wcp.lib.SwerveInverseKinematics;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.PathFollower;
import com.wcp.lib.util.SynchronousPIDF;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Swerve extends Subsystem {

    public static Swerve instance = null;

    public static Swerve getInstance() {// if doesnt have an instance of swerve will make a new one
        if (instance == null)
            instance = new Swerve();
        return instance;
    }


    SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    List<SwerveDriveModule> modules;
    Translation2d aimingVector  = new Translation2d();

    Translation2d translationVector;
    public double rotationScalar;
    public double autox;
    public double autoy;
    public double autorotate;
    public double fieldcolor;
    double scaleFactor;
    boolean aimFinished;
    double rotationalVel;
    boolean trajectoryStarted = false;
    boolean trajectoryFinished= false;
    double speed;
    boolean useAllianceColor;
    Pigeon gyro;
    PathFollower pathFollower;
    Pose2d pose = new Pose2d(new Translation2d(5,5),new Rotation2d());
    Pose2d drivingpose = new Pose2d();
    PathPlannerTrajectory trajectoryDesired;
    List<Translation2d> moduleVectors;
    public final Timer aimTimer = new Timer();
    final double translationDeadband = 0.1;
    final double rotationDeadband = 0.1;
    private boolean robotCentric = false;
    double desiredRotationScalar;
    List<SwerveDriveModule> positionModules;
	double distanceTraveled;
	double currentVelocity = 0;
	double lastUpdateTimestamp = 0;

    SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
    public HeadingController headingController = new HeadingController();


 boolean pathStarted;
  double distance;
  int bestScore;
  int offset;
  double xError;
  double yError;
  boolean ran = false;
  boolean isAiming = false;

  PIDController thetaController;
  PIDController advanceController;

  SynchronousPIDF rPID;
  double bestDistance;


  double lastTimeStamp = 0;
  double Roboty;
  double Robotx;
  Vision vision = Vision.getInstance();
  Gyro pigeon = Pigeon.getInstance();
  double xERROR;
  double yERROR;
  boolean targetTracked;

    public Swerve() {
        // setting values for the modules, id, and wether or not the encoders are
        // flipped
        frontRightModule = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE, 0,
                Constants.kFrontRightStartingEncoderPosition, Constants.kFrontRightPosition, true,Constants.mFrontRightPosition);
        frontLeftModule = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE, 1,
                Constants.kFrontLeftStartingEncoderPosition, Constants.kFrontLeftPosition, true,Constants.mFrontLeftPosition);
        rearLeftModule = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE, 2,
                Constants.kRearLeftStartingEncoderPosition, Constants.kRearLeftPosition, true,Constants.mRearLeftPosition);
        rearRightModule = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE, 3,
                Constants.kRearRightStartingEncoderPosition, Constants.kRearRightPosition, true,Constants.mRearRightPosition);
        modules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

        // sets which ways the modules turn
        frontRightModule.invertDriveMotor(TalonFXInvertType.Clockwise);
        frontLeftModule.invertDriveMotor(TalonFXInvertType.CounterClockwise);
        rearLeftModule.invertDriveMotor(TalonFXInvertType.CounterClockwise);
        rearRightModule.invertDriveMotor(TalonFXInvertType.Clockwise);
        positionModules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);
        distanceTraveled = 0;
        pathFollower = PathFollower.getInstance();
        gyro = Pigeon.getInstance();
    }


    public void setTrajectory(PathPlannerTrajectory trajectory){
        trajectoryDesired= trajectory;
    }

    private enum State {
        MANUAL,
        TRAJECTORY,
        OFF,
        OBJECT,
        SCORE,
        BALANCE;
    } 
    private State currentState = State.MANUAL;
    public State getState() {
        return this.currentState;
    }
    public void setState(State desiredState) {
        currentState = desiredState;
    }
    public void startPath(double speed, boolean useAllianceColor){
        trajectoryStarted = true;
        this.speed = speed;
        this.useAllianceColor = useAllianceColor;
        pathFollower.startTimer();

    }
    //
    public void sendInput(double x, double y, double rotation) {
        translationVector = new Translation2d(x, y);// makes vector to inputs
        // sets rotation to zero if it doesnt pass deadband
        if (Math.abs(rotation) <= rotationDeadband) {
            rotation = 0;
        }
        // disables heading controller if rotation is 0 and rotation scalar is not equal
        // to 0
        if (rotation == 0 && rotationScalar != 0) {
            headingController.disableHeadingController(true);
        }

        //
        rotationScalar = rotation;
        final double scaleValue = 1.5;
        double inputMagnitude = translationVector.norm();
        inputMagnitude = Math.pow(inputMagnitude, scaleValue);
        inputMagnitude = Util.deadband(translationDeadband, inputMagnitude);
        if (translationVector.norm() <= translationDeadband) {
            translationVector = new Translation2d();
        }
        rotationScalar *= 0.01;
        if (translationVector.norm() <= translationDeadband && Math.abs(rotation) <= rotationDeadband) {// deadbands
            this.commandModuleDrivePowers(0);// no power
        } else {
            this.update();// command modules


        }
        this.update();// command modules

    }
    public boolean isFinishedTrajectory(){
        trajectoryStarted = false;
        return pathFollower.isFinished();
    }
    public void updateTrajectory(){
        Pose2d desiredPose = pathFollower.getDesiredPose2d(useAllianceColor,speed, getPose());
        scaleFactor = speed;
        targetHeading=desiredPose.getRotation().inverse();
        targetFollowTranslation = desiredPose.getTranslation();
        trajectoryFinished = false;

    }


    public boolean inAimRange(){
        if (DriverStation.getAlliance() == Alliance.Blue)
            return getPose().getTranslation().getX() < 2.8;
        else
            return getPose().getTranslation().getX() >14;
    }


    public void parkMode() {// makes it thin it rotating but cuts off drive power
        rotationScalar = .5;
        rotationScalar *= 0.01;
        this.update();
        this.commandModuleDrivePowers(0);
    }

    public void setModuleStates(SwerveModuleState[] moduleState) {// sets modules based off of a list
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setModuleStates(moduleState[i]);
        }

    }
  
    public void commandModules(List<Translation2d> moduleVectors) {// for every module will optimize rotation and set
                                                                    // angle and drive speed
        this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveOpenLoop(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(moduleVectors.get(i).norm());

            }
        }
    }

    public void commandModuleDrivePowers(double power) {// sets power
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveOpenLoop(power);
        }
    }



    private Translation2d targetFollowTranslation = new Translation2d();
    private Rotation2d targetHeading = new Rotation2d();
    private boolean reverseXPID;
    
    SynchronousPIDF xPID = new SynchronousPIDF(1, 0.0, 0);
    SynchronousPIDF yPID = new SynchronousPIDF(1, 0.0, 0);
    private double lastTimestamp = Timer.getFPGATimestamp();
    public void followTranslation2d(Translation2d translation2d, Rotation2d targetRobotHeading, double speed) {
        setState(State.TRAJECTORY);
        scaleFactor = speed;
        if (translation2d.getX()<0){
            reverseXPID = true;
        }
        else {
            reverseXPID = false;
        }

        targetHeading = targetRobotHeading;
        targetFollowTranslation = translation2d;
        trajectoryFinished = false;


    }

    public void resetPose(Pose2d newPose){
		modules.forEach((m) -> m.resetPose(newPose));
	}
    /** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = getRobotHeading();
		
		double averageDistance = 0.0;
		double[] distances = new double[4];
		for(SwerveDriveModule m : positionModules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse()).norm();
			distances[m.moduleID] = distance;
			averageDistance += distance;
		}
		averageDistance /= positionModules.size();
		
		int minDevianceIndex = 0;
		double minDeviance = Units.inchesToMeters(100);
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		for(SwerveDriveModule m : positionModules){
				double deviance = Math.abs(distances[m.moduleID] - averageDistance);
				if(deviance < minDeviance){
					minDeviance = deviance;
					minDevianceIndex = m.moduleID;
				}
				if(deviance <= 10000){
					modulesToUse.add(m);
				}
			}
		
		if(modulesToUse.isEmpty()){
			modulesToUse.add(modules.get(minDevianceIndex));
		}
		


		//SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().getX();
			y += m.getEstimatedRobotPose().getTranslation().getY();
		}
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
        Logger.getInstance().recordOutput("delta pose", deltaPos);
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}
    public void setTrajectory (PathPlannerTrajectory trajectory,List<Translation2d> eventTimings, List<Command> events, List<Double> waitTiming) {
        pathFollower.setTrajectory(trajectory);
        pathFollower.setEventTimings(eventTimings);
        pathFollower.setEvents(events);
        pathFollower.setWaitTimings(waitTiming);
        resetOdometry(pathFollower.getStart(),Rotation2d.fromDegrees(pathFollower.getrotation())); 
        resetGryo(pathFollower.getStartRotation());
        Logger.getInstance().recordOutput("startRotation", pathFollower.getStart());
        setTrajectory(trajectory);
        resetTimer();
    
    }
    public void resetOdometry (Pose2d newpose, Rotation2d rotation){
       Pose2d newpose2 = new Pose2d(newpose.getTranslation(),rotation);
		modules.forEach((m) -> m.resetPose(newpose2));

   }
    public void resetTimer (){
        PathFollower.getInstance().resetTimer();
    }
    public Translation2d updateFollowedTranslation2d(double timestamp) {
        double dt = timestamp - lastTimestamp;
        Logger.getInstance().recordOutput("desiredPoseInches",new Pose2d(targetFollowTranslation, targetHeading).toWPI());

        Translation2d currentRobotPositionFromStart = pose.getTranslation();
        double xError = xPID.calculate(targetFollowTranslation.getX() -  currentRobotPositionFromStart.getX(), dt);
        double yError = yPID.calculate(targetFollowTranslation.getY() - currentRobotPositionFromStart.getY(), dt);
        Logger.getInstance().recordOutput("xError", targetFollowTranslation.getX() -  currentRobotPositionFromStart.getX());
        Logger.getInstance().recordOutput("yError", targetFollowTranslation.getY() - currentRobotPositionFromStart.getY());
        Logger.getInstance().recordOutput("rotationErorr", getRobotHeading().getDegrees()-targetHeading.getDegrees());

        lastTimestamp = timestamp;
        if(Math.abs(xError+yError)/2<.05&&PathFollower.getInstance().isFinished()) {
            setState(State.OFF);
            trajectoryFinished = true;

            return new Translation2d();
        }
        return new Translation2d(xError, -yError);

    }
    
    



    public void zeroModules() {
        modules.forEach((m) -> {
            m.resetModulePositionToAbsolute();
        });
    }

    public boolean isTrajectoryFollowed(){
        setState(State.MANUAL);
        return trajectoryFinished;
    }
    public void resetTrajectoryFollowed(){
         trajectoryFinished = false;
    }



    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }
    @Override
    public void update() {
        double timeStamp = Timer.getFPGATimestamp();
        drivingpose = Pose2d.fromRotaiton(getRobotHeading());

        switch(currentState){
            case MANUAL:
                double rotationCorrection =  headingController.updateRotationCorrection(drivingpose.getRotation(), timeStamp);
                if(translationVector.norm() == 0 || rotationScalar != 0) {
                    rotationCorrection = 0;
                }
                SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
                commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationScalar + rotationCorrection, drivingpose, robotCentric));
            break;

            case SCORE:
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
                commandModules(inverseKinematics.updateDriveVectors(aimingVector, rotationCorrection, drivingpose, robotCentric));
            break;

            case BALANCE:
                headingController.setTargetHeading(targetHeading);;
                rotationCorrection = headingController.getRotationCorrection(targetHeading, timeStamp);
                commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, drivingpose, robotCentric));
            break;

            case TRAJECTORY:
                Translation2d translationCorrection = updateFollowedTranslation2d(timeStamp).scale(1);
                headingController.setTargetHeading(targetHeading);
                 rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;    
                Logger.getInstance().recordOutput("targetHeading", targetHeading.getDegrees());
                commandModules(inverseKinematics.updateDriveVectors(translationCorrection, rotationCorrection, pose, robotCentric));
            break;

            case OBJECT:
                commandModules(inverseKinematics.updateDriveVectors(aimingVector, rotationScalar, drivingpose, robotCentric));
            break;

            case OFF:
                commandModules(inverseKinematics.updateDriveVectors(new Translation2d(), 0, drivingpose, robotCentric));
            break;
                

        }

    }

    public boolean balance(){
        double x = 0;
        double scalar = .012;
            if(pigeon.getPitch()>1){
              x = -scalar * pigeon.getPitch();
            }else if(pigeon.getPitch() < -1){
              x = -scalar * pigeon.getPitch();
            }else{
              x = 0;
            }
            translationVector = new Translation2d(x,0);
            if(DriverStation.getAlliance() == Alliance.Blue){
                targetHeading = new Rotation2d();
            }
            else{
                targetHeading = new Rotation2d(180);
            }
            return Math.abs(x)<.03;
          
    }
    public void Aim(Translation2d aimingVector, double scalar){
        currentState = State.OBJECT;
        this.aimingVector = aimingVector;
        this.rotationScalar = scalar;
        update();
    }
    public void Aim(Translation2d aimingVector,Rotation2d rotation){
        currentState = State.SCORE;// SETS HEADING TO 0or 180
        this.aimingVector = aimingVector;
        targetHeading = rotation;
        update();

    }

    public void updateOdometry(double timestamp) {// uses sent input to commad modules and correct for rotatinol drift

    lastUpdateTimestamp = timestamp;

    }
    public Pose2d getPose(){
        return pose;
    }
    public void followTrajectory() {
        if (currentState == State.TRAJECTORY){
        
        double timestamp = Timer.getFPGATimestamp();
        if(currentState == State.TRAJECTORY) {
            Translation2d translationCorrection = updateFollowedTranslation2d(timestamp).scale(1);
            headingController.setTargetHeading(targetHeading);
            double rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timestamp);
            desiredRotationScalar = rotationCorrection;    
            Logger.getInstance().recordOutput("targetHeading", targetHeading.getDegrees());
            commandModules(inverseKinematics.updateDriveVectors(translationCorrection, rotationCorrection, pose, robotCentric));
        }
    }
    }
    public void startTrjectory(){
        if(trajectoryStarted){
            updateTrajectory();
        }
        setState(State.TRAJECTORY);

    }
    public double getRotationalVelSIM(){

        return desiredRotationScalar;
    }

    
    


    public void fieldzeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2d.fromDegrees(-180));
        gyro.setAngle(-180);
    }
    
  public double getAverageVel(){
    double average=0;
    for (int i= 0; i<modules.size(); i++){
      average = average+ modules.get(i).currentvel();
    }
    return average/4;
}

    @Override
    public void readPeriodicInputs() {
        modules.forEach((m) -> {
            m.readPeriodicInputs();
        });
    }

    @Override
    public void writePeriodicOutputs() {
        modules.forEach((m) -> {
            m.writePeriodicOutputs();
        });
    }
    public void resetGryo(double angle){
        headingController.setTargetHeading(Rotation2d.fromDegrees(angle));
        gyro.setAngle(angle);
    }

    public void zeroSwerve() {// zeros gyro
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        gyro.setAngle(0);
        resetPose(new Pose2d(new Translation2d(1.9,4.52),Rotation2d.fromDegrees(0)));
    }


    public void resetEncoders() {// zeros encoders
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).resetEncoders();
        }

    }

    public Request balanceRequest(){
        return new Request() {
            @Override
                public void act(){
                    setState(State.BALANCE);
                    balance();
                }
            @Override
                public boolean isFinished(){
                    if(balance()) setState(State.MANUAL);
                    return balance();
                }
        };
    }

    public Request objectTartgetRequest(){
        return new Request() {


            @Override
            public void act() {
                goToObject();
            }
            @Override
            public boolean isFinished(){
                if(goToObject()){
                    setState(State.MANUAL);
                }
                return goToObject();
            }
            
        };
    }
    public Request goToNodeRequest(int node){
        return new Request() {
            @Override
            public void act(){
                goToNodeRequest(node);
            }
            @Override
            public boolean isFinished(){
                if(aimFinished) resetOffset();
                return aimFinished;
            }
        };

    }
    


    public Request aimStateRequest(boolean snapUp, boolean snapDown){
        return new Request() {

            @Override
                public void initialize() {
                    aimFinished = false;
                }
            
            @Override
                public void act() {
                    setState(State.SCORE);
                    aimAtScore(snapUp, snapDown);
                }

            @Override
                public boolean isFinished(){
                    if(aimFinished) resetOffset();
                    return aimFinished;
                }
        };        
    }

    public Request goToChuteRequest(){
        return new Request(){

            @Override
                public void act(){
                    if(DriverStation.getAlliance() == Alliance.Blue){
                          
                    }
                }
        }
    }

  
  
  public void targetNode(int scoringNode){
    double Roboty = getPose().getTranslation().getY();
    double Robotx = getPose().getTranslation().getX();
    double rotation = 0;
    if(DriverStation.getAlliance() == Alliance.Blue){
      rotation = 180;
    }
    double currentTime  = Timer.getFPGATimestamp();
    double dt = currentTime-lastTimeStamp; 

    xPID.setSetpoint(2);
    yPID.setSetpoint(Constants.scoresY.get(scoringNode));
    double xError = xPID.calculate(Robotx, dt);
    double yError = yPID.calculate(Roboty, dt);
    Logger.getInstance().recordOutput("yerror", yError);
    Logger.getInstance().recordOutput("xerror", xError);

    Aim(new Translation2d(-xError, yError), Rotation2d.fromDegrees(rotation));
    if(Math.abs(xError)<.03 && Math.abs(yError)<.3) aimTimer.start();
    if(aimTimer.get()>.2){
        aimFinished = true;
    }
    lastTimeStamp = currentTime;
  }
  public void aimAtScore(boolean snapDown,boolean snapUp){
    double Roboty = getPose().getTranslation().getY();
    bestDistance = 11111;
  if(!ran){
   for(int i = 0; i < Constants.scoresY.size(); i++){//finds closest scoring node
    distance = Math.abs(Constants.scoresY.get(i)-Roboty);
        if(bestDistance>distance){//if closer than previous closest
        bestDistance = distance;
        bestScore = i;//sets target to closest plus the user inputed offset
      }
    
    
        if(i == Constants.scoresY.size()-1){
        ran = true;
        }
        
    }
  }

  if(snapUp && (bestScore+offset + 1 < Constants.scoresY.size())){//if wants to move up and isnt at 10 than move up
    offset++;//sets desired scoring station to snap to the next one up
  }else if(snapDown&& (bestScore+offset> 0)){//if wants to move down and isnt at zero than move down
    offset--;//sets desired scoring station to snap to the next one down
  }
  Logger.getInstance().recordOutput("bestDistance", offset);


  if(DriverStation.getAlliance() == Alliance.Blue){
    if(Robotx < 2.5){
      targetNode(bestScore+offset);
      Logger.getInstance().recordOutput("BEst", bestScore);

    }else{
      targetNode(bestScore);
      Logger.getInstance().recordOutput("BEst", bestScore);

    }
      
  }
  else {
    if(Robotx < 14.02){
      targetNode(bestScore+offset);
      Logger.getInstance().recordOutput("BEst", bestScore);

    }else{
      targetNode(bestScore);
      Logger.getInstance().recordOutput("BEst", bestScore);
  }
  }
  
  }
  public boolean goToObject(){
    //makes sure we have can see a target
    vision.setPipeline(Constants.VisionConstants.CONE_PIPELNE);
    if(!vision.hasTarget()){//if we cant see a cone we will look for a cube
      vision.setPipeline(Constants.VisionConstants.CUBE_PIPELINE);
      if(!vision.hasTarget()){
        return false;//if we cant see a cube we will exit the function becuase we dont have anywhere to go
      }
    }
    //gets error
    double xERROR = thetaController.calculate(vision.getX(),0);
    double yERROR= advanceController.calculate(vision.getY(),-5);
    //corrects for error
    Aim(new Translation2d(xERROR,yERROR),0);
    if(Math.abs(new Translation2d(xERROR,yERROR).norm())<.3){
        return true;
    }else{
        return false;
    }

  }

  public boolean isAiming(){
    return !aimFinished;
  }

  public Request openLoopRequest(Translation2d x, double r){
       return new Request(){

            @Override
            public void act() {
                currentState = State.MANUAL;
                sendInput(x.getX(), x.getY(), r);
                
            }
            
        };

  }
  public void resetOffset(){
    offset = 0;
    ran = false;
    aimTimer.reset();
    aimTimer.stop();
  }

  public void goToObject(boolean cube){
    Rotation2d heading = Rotation2d.fromDegrees(pigeon.getAngle());
    if(cube){
      vision.setPipeline(Constants.VisionConstants.CUBE_PIPELINE);
    }
    else{
      vision.setPipeline(Constants.VisionConstants.CONE_PIPELNE);
    }
    double xSetPoint = (.1*heading.getCos());
    double ySetPoint = (.1*heading.getSin());
    double xError = xPID.calculate(vision.getDistanceToGroundObject()*heading.getCos(),xSetPoint);
    double yError = yPID.calculate(vision.getDistanceToGroundObject()*heading.getSin(),ySetPoint);
    double thetaControl = rPID.calculate(vision.getY(), 0);

    Aim(new Translation2d(xError,yERROR), thetaControl);

  }


    @Override
    public void outputTelemetry() {// outputs telemetry
        modules.forEach((m) -> {
            m.outputTelemetry();
        });
        SmartDashboard.putNumber("Robot Heading", getRobotHeading().getDegrees());
        SmartDashboard.putNumber("Radians Heading", -getRobotHeading().getRadians());
        Logger.getInstance().recordOutput("Odometry",pose.toWPI());


    }

    @Override
    public void stop() {// stops everything
        setState(State.MANUAL);
        translationVector = new Translation2d();
        rotationScalar = 0;

        update();
        commandModuleDrivePowers(0);
    }

}
