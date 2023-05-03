// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.Conversions;
import com.wcp.lib.HeadingController;
import com.wcp.lib.SwerveInverseKinematics;
import com.wcp.lib.geometry.Pose2dd;
import com.wcp.lib.geometry.Rotation2dd;
import com.wcp.lib.geometry.Translation2dd;
import com.wcp.lib.util.ScuffedPathPlanner;
import com.wcp.lib.util.SynchronousPIDF;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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

    Translation2dd translationVector;
    public double rotationScalar;
    public double autox;
    public double autoy;
    public double autorotate;
    public double fieldcolor;
    double scaleFactor;
    double rotationalVel;
    boolean trajectoryStarted = false;
    boolean trajectoryFinished= false;
    double speed;
    boolean useAllianceColor;
    Pigeon gyro;
    ScuffedPathPlanner scuffedPathPlanner;
    Pose2dd pose = new Pose2dd(new Translation2dd(5,5),new Rotation2dd());
    Pose2dd drivingpose = new Pose2dd();
    PathPlannerTrajectory trajectoryDesired;
    List<Translation2dd> moduleVectors;
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



        scuffedPathPlanner = ScuffedPathPlanner.getInstance();
        gyro = Pigeon.getInstance();
        gyro.setAngle(0);

        modules.forEach((m) -> m.resetPose(new Pose2dd(new Translation2dd(5,5),new Rotation2dd())));


    }


    public void setTrajectory(PathPlannerTrajectory trajectory){
        trajectoryDesired= trajectory;
    }

    private enum State {
        MANUAL, VECTOR,OFF;
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
        scuffedPathPlanner.startTimer();

    }
    //
    public void sendInput(double x, double y, double rotation) {
        setState(State.MANUAL);
        translationVector = new Translation2dd(x, y);// makes vector to inputs
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
            translationVector = new Translation2dd();
        }
        rotationScalar *= 0.01;
        if (translationVector.norm() <= translationDeadband && Math.abs(rotation) <= rotationDeadband) {// deadbands
            this.commandModuleDrivePowers(0);// no power
        } else {
            this.update(Timer.getFPGATimestamp());// command modules


        }
        this.update(Timer.getFPGATimestamp());// command modules

    }
    public boolean isFinishedTrajectory(){
        trajectoryStarted = false;
        return scuffedPathPlanner.isFinished();
    }
    public void updateTrajectory(){
        Pose2dd desiredPose = scuffedPathPlanner.getDesiredPose2dd(useAllianceColor,speed, Util.Poseconvert2ddto2d( getPose()));
        Logger.getInstance().recordOutput("desiredPosition", Util.Poseconvert2ddto2d(desiredPose));
        scaleFactor = speed;
        targetHeading=desiredPose.getRotation().inverse();
        targetFollowTranslation = desiredPose.getTranslation();
        trajectoryFinished = false;

    }
    public BooleanSupplier TrajectoryisFinished (){
        return new BooleanSupplier() {

            @Override
            public boolean getAsBoolean() {
                if (isTrajectoryFollowed()){
                    trajectoryStarted = false;
                }
                // TODO Auto-generated method stub
                return isTrajectoryFollowed();
            }
    };
};


    public void parkMode() {// makes it thin it rotating but cuts off drive power
        rotationScalar = .5;
        rotationScalar *= 0.01;
        this.update(Timer.getFPGATimestamp());
        this.commandModuleDrivePowers(0);
    }

    public void setModuleStates(SwerveModuleState[] moduleState) {// sets modules based off of a list
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setModuleStates(moduleState[i]);
        }

    }
  
    public void commandModules(List<Translation2dd> moduleVectors) {// for every module will optimize rotation and set
                                                                    // angle and drive speed
        this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2dd.fromDegrees(modules.get(i).getModuleAngle()))) {
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



    private Translation2dd targetFollowTranslation = new Translation2dd();
    private Rotation2dd targetHeading = new Rotation2dd();
    private boolean reverseXPID;
    
    SynchronousPIDF xPID = new SynchronousPIDF(1, 0.0, 0);
    SynchronousPIDF yPID = new SynchronousPIDF(1, 0.0, 0);
    private double lastTimestamp = Timer.getFPGATimestamp();
    public void followTranslation2d(Translation2dd translation2d, Rotation2dd targetRobotHeading, double speed) {
        setState(State.VECTOR);
        scaleFactor = speed;
        if (translation2d.x()<0){
            reverseXPID = true;
        }
        else {
            reverseXPID = false;
        }

        targetHeading = targetRobotHeading;
        targetFollowTranslation = translation2d;
        trajectoryFinished = false;


    }
    public void resetPose(){
		modules.forEach((m) -> m.resetPose());
	}
    /** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp){
		double x = 0.0;
		double y = 0.0;
		Rotation2dd heading = getRobotHeading();
		
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
				if(deviance <= 0.05){
					modulesToUse.add(m);
				}
			}
		
		if(modulesToUse.isEmpty()){
			modulesToUse.add(modules.get(minDevianceIndex));
		}
		


		//SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		Pose2dd updatedPose = new Pose2dd(new Translation2dd(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
        Logger.getInstance().recordOutput("delta pose", deltaPos);
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
	}
    public void setTrajectory (PathPlannerTrajectory trajectory,List<Translation2d> eventTimings, List<Command> events, List<Double> waitTiming) {
        scuffedPathPlanner.setTrajectory(trajectory);
        scuffedPathPlanner.setEventTimings(eventTimings);
        scuffedPathPlanner.setEvents(events);
        scuffedPathPlanner.setWaitTimings(waitTiming);
        resetOdometry(Util.Poseconvert2dto2dd( scuffedPathPlanner.getStart()),Rotation2dd.fromDegrees(scuffedPathPlanner.getrotation())); 
        resetGryo(scuffedPathPlanner.getStartRotation());
        Logger.getInstance().recordOutput("startRotation", scuffedPathPlanner.getStartRotation());
        setTrajectory(trajectory);
        resetTimer();
    
    }
    public void resetOdometry (Pose2dd newpose, Rotation2dd rotation){
       Pose2dd newpose2 = new Pose2dd(newpose.getTranslation(),rotation);
		modules.forEach((m) -> m.resetPose(newpose2));

   }
    public void resetTimer (){
        ScuffedPathPlanner.getInstance().resetTimer();
    }
    public Translation2dd updateFollowedTranslation2d(double timestamp) {
        double dt = timestamp - lastTimestamp;
        Logger.getInstance().recordOutput("poseinches", Util.Poseconvert2ddto2d(pose));
        Logger.getInstance().recordOutput("desiredPoseInches", Util.Poseconvert2ddto2d(new Pose2dd(targetFollowTranslation, targetHeading)));

        Translation2dd currentRobotPositionFromStart = pose.getTranslation();
        double xError = xPID.calculate(targetFollowTranslation.x() -  currentRobotPositionFromStart.x(), dt);
        double yError = yPID.calculate(targetFollowTranslation.y() - currentRobotPositionFromStart.y(), dt);
        Logger.getInstance().recordOutput("xError", targetFollowTranslation.x() -  currentRobotPositionFromStart.x());
        Logger.getInstance().recordOutput("yError", targetFollowTranslation.y() - currentRobotPositionFromStart.y());
        Logger.getInstance().recordOutput("rotationErorr", getRobotHeading().getDegrees()-targetHeading.getDegrees());

        lastTimestamp = timestamp;
        if(Math.abs(xError+yError)/2<.01&&ScuffedPathPlanner.getInstance().isFinished()) {
            setState(State.OFF);
            trajectoryFinished = true;

            return new Translation2dd();
        }
        return new Translation2dd(xError, -yError);

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



    public Rotation2dd getRobotHeading() {
        return Rotation2dd.fromDegrees(gyro.getAngle());
    }

    public void update(double timestamp) {// uses sent input to commad modules and correct for rotatinol drift
                drivingpose = Pose2dd.fromRotaiton(getRobotHeading());


        if(currentState == State.MANUAL) {
            double rotationCorrection = headingController.updateRotationCorrection(getRobotHeading(), timestamp);

            if (translationVector.norm() == 0 || rotationScalar != 0) {
                rotationCorrection = 0;
            }
            SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
            commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationScalar + rotationCorrection,
                    drivingpose, robotCentric));// where the magic happens
        }

    }
    public void updateOdometry(double timestamp) {// uses sent input to commad modules and correct for rotatinol drift

    lastUpdateTimestamp = timestamp;

    }
    public Pose2dd getPose(){
        return pose;
    }
    public void followTrajectory() {
        if (currentState == State.VECTOR){
        
        double timestamp = Timer.getFPGATimestamp();
        if(currentState == State.VECTOR) {
            Translation2dd translationCorrection = updateFollowedTranslation2d(timestamp).scale(1);
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
        setState(State.VECTOR);

    }
    public double getRotationalVelSIM(){

        return desiredRotationScalar;
    }
    


    public void fieldzeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2dd.fromDegrees(-180));
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
        headingController.setTargetHeading(Rotation2dd.fromDegrees(angle));
        gyro.setAngle(angle);
    }

    public void zeroSwerve() {// zeros gyro
        headingController.setTargetHeading(Rotation2dd.fromDegrees(0));
        gyro.setAngle(0);
    }

    public void resetEncoders() {// zeros encoders
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).resetEncoders();
        }

    }

    @Override
    public void outputTelemetry() {// outputs telemetry
        modules.forEach((m) -> {
            m.outputTelemetry();
        });
        SmartDashboard.putNumber("Robot Heading", getRobotHeading().getDegrees());
        SmartDashboard.putNumber("Radians Heading", -getRobotHeading().getRadians());
        Logger.getInstance().recordOutput("Odometry",Util.Poseconvert2ddto2d(pose));


    }

    @Override
    public void stop() {// stops everything
        setState(State.MANUAL);
        translationVector = new Translation2dd();
        rotationScalar = 0;

        update(Timer.getFPGATimestamp());
        commandModuleDrivePowers(0);
    }

}
