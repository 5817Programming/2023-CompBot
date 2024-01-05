// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.Conversions;
import com.wcp.lib.HeadingController;
import com.wcp.lib.KalmanFilter;
import com.wcp.lib.SwerveInverseKinematics;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.HeavilyInspired.Node;
import com.wcp.lib.util.PathFollower;
import com.wcp.lib.util.PathGenerator;
import com.wcp.lib.util.SynchronousPIDF;
import com.wcp.lib.util.Util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Swerve extends Subsystem {
    Timer visionUpdateTimer = new Timer();

    public static Swerve instance = null;

    public static Swerve getInstance() {// if doesnt have an instance of swerve will make a new one
        if (instance == null)
            instance = new Swerve();
        return instance;
    }

    SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    List<SwerveDriveModule> modules;
    Translation2d aimingVector = new Translation2d();

    Translation2d translationVector = new Translation2d();
    public double rotationScalar = 0;
    public double autox;
    public double autoy;
    public double autorotate;
    public double fieldcolor;
    double scaleFactor;
    boolean aimFinished = false;
    double rotationalVel;
    boolean trajectoryStarted = false;
    boolean trajectoryFinished = false;
    double speed;
    double percentToRotate;
    boolean useAllianceColor;
    Pigeon gyro;
    PathFollower pathFollower;
    Pose2d pose = new Pose2d(new Translation2d(5, 5), new Rotation2d());
    Pose2d drivingpose = new Pose2d();
    PathPlannerTrajectory trajectoryDesired;
    double chuteLastTimeStamp;
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
    double errorFromScore = 0;

    private Translation2d targetFollowTranslation = new Translation2d();
    private Rotation2d targetHeading = new Rotation2d();

    SynchronousPIDF xOPID = new SynchronousPIDF(1, 0.0, 0);
    SynchronousPIDF yOPID = new SynchronousPIDF(1, 0.0, 0);
    SynchronousPIDF xVPID = new SynchronousPIDF(.01, 0.005, 0.01);
    SynchronousPIDF yVPID = new SynchronousPIDF(.5, 0.01, 0);
    SynchronousPIDF aVPID = new SynchronousPIDF(.025, 0.01, 0);

    private double lastTimestamp = Timer.getFPGATimestamp();
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
    Gyro pigeon = Pigeon.getInstance();
    double xERROR;
    double yERROR;
    boolean targetTracked;

    KalmanFilter filter = new KalmanFilter();

    public Swerve() {
        visionUpdateTimer.start();

        // setting values for the modules, id, and wether or not the encoders are
        // flipped
        frontRightModule = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE, 0,
                Constants.kFrontRightStartingEncoderPosition, Constants.kFrontRightPosition, true,
                Constants.mFrontRightPosition);
        frontLeftModule = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE, 1,
                Constants.kFrontLeftStartingEncoderPosition, Constants.kFrontLeftPosition, true,
                Constants.mFrontLeftPosition);
        rearLeftModule = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE, 2,
                Constants.kRearLeftStartingEncoderPosition, Constants.kRearLeftPosition, true,
                Constants.mRearLeftPosition);
        rearRightModule = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE, 3,
                Constants.kRearRightStartingEncoderPosition, Constants.kRearRightPosition, true,
                Constants.mRearRightPosition);
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
        visionUpdateTimer.start();

    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        trajectoryFinished = false;
        pathFollower.setTrajectory(trajectory);
        Pose2d newpose = (pathFollower.getInitial(trajectory));
        modules.forEach((m) -> m.resetPose(new Pose2d(newpose.getTranslation(), new Rotation2d())));
        pigeon.setAngle(newpose.getRotation().getDegrees());

    }

    public enum State {
        MANUAL,
        TRAJECTORY,
        OFF,
        OBJECT,
        SCORE,
        SNAP,
        BALANCE;
    }

    private State currentState = State.MANUAL;

    public State getState() {
        return this.currentState;
    }

    public void setState(State desiredState) {
        currentState = desiredState;
    }

    public void startPath(double speed, boolean useAllianceColor) {
        trajectoryStarted = true;
        this.speed = speed;
        this.useAllianceColor = useAllianceColor;
        pathFollower.startTimer();

    }

    //
    public void sendInput(double x, double y, double rotation) {
        translationVector = new Translation2d(x, y);// makes vector to input
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

    public boolean isFinishedTrajectory() {
        trajectoryStarted = false;
        return pathFollower.isFinished();
    }

    public void updateTrajectory() {
        Pose2d desiredPose = pathFollower.getDesiredPose2d(useAllianceColor, speed, getPose(), percentToRotate);
        scaleFactor = speed;
        targetHeading = desiredPose.getRotation().inverse();
        targetFollowTranslation = desiredPose.getTranslation();
    }

    public boolean inAimRange() {
        if (DriverStation.getAlliance() == Alliance.Blue)
            return getPose().getTranslation().getX() < 2.8;
        else
            return getPose().getTranslation().getX() > 14;
    }

    public void parkMode() {// makes it thin it rotating but cuts off drive power
        rotationScalar = .5;
        rotationScalar *= 0.01;
        this.update();
        this.commandModuleDrivePowers(0);
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

    public void resetPose(Pose2d newPose) {
        modules.forEach((m) -> m.resetPose(newPose));
    }

    /** The tried and true algorithm for keeping track of position */
    public synchronized void updatePose(double timestamp) {

        double x = 0.0;
        double y = 0.0;
        Rotation2d heading = getRobotHeading();

        double averageDistance = 0.0;
        double[] distances = new double[4];
        for (SwerveDriveModule m : positionModules) {
            m.updatePose(heading);
            double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse())
                    .norm();
            distances[m.moduleID] = distance;
            averageDistance += distance;
        }
        averageDistance /= positionModules.size();

        int minDevianceIndex = 0;
        double minDeviance = Units.inchesToMeters(100);
        List<SwerveDriveModule> modulesToUse = new ArrayList<>();
        for (SwerveDriveModule m : positionModules) {
            double deviance = Math.abs(distances[m.moduleID] - averageDistance);
            if (deviance < minDeviance) {
                minDeviance = deviance;
                minDevianceIndex = m.moduleID;
            }
            if (deviance <= 10000) {
                modulesToUse.add(m);
            }
        }

        if (modulesToUse.isEmpty()) {
            modulesToUse.add(modules.get(minDevianceIndex));
        }

        // SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for (SwerveDriveModule m : modulesToUse) {
            x += m.getEstimatedRobotPose().getTranslation().getX();
            y += m.getEstimatedRobotPose().getTranslation().getY();
        }
        Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
        Translation2d deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse());
        Logger.getInstance().recordOutput("delta pose", deltaPos.getNorm());
        distanceTraveled += deltaPos.getNorm();
        currentVelocity = deltaPos.getNorm() / (timestamp - lastUpdateTimestamp);

        pose = updatedPose;


        modules.forEach((m) -> m.resetPose(pose));
        lastUpdateTimestamp = timestamp;
    }

    public void resetOdometry(Pose2d newpose, Rotation2d rotation) {
        Pose2d newpose2 = new Pose2d(newpose.getTranslation(), rotation);
        modules.forEach((m) -> m.resetPose(newpose2));

    }

    public void resetTimer() {
        PathFollower.getInstance().resetTimer();
    }

    public Translation2d updateFollowedTranslation2d(double timestamp) {
        double dt = timestamp - lastTimestamp;
        Logger.getInstance().recordOutput("desiredPoseInches",
                new Pose2d(targetFollowTranslation, targetHeading).toWPI());

        Translation2d currentRobotPositionFromStart = pose.getTranslation();
        double xError = xOPID.calculate(targetFollowTranslation.getX() - currentRobotPositionFromStart.getX(), dt);
        double yError = yOPID.calculate(targetFollowTranslation.getY() - currentRobotPositionFromStart.getY(), dt);
        Logger.getInstance().recordOutput("xError",
                targetFollowTranslation.getX() - currentRobotPositionFromStart.getX());
        Logger.getInstance().recordOutput("yError",
                targetFollowTranslation.getY() - currentRobotPositionFromStart.getY());
        Logger.getInstance().recordOutput("rotationErorr", getRobotHeading().getDegrees() - targetHeading.getDegrees());

        lastTimestamp = timestamp;
        if (Math.abs(xError + yError) / 2 < .1 && PathFollower.getInstance().isFinished()) {
            setState(State.OFF);
            trajectoryFinished = true;
            return new Translation2d();
        }
        Logger.getInstance().recordOutput("traj finished", trajectoryFinished);
        return new Translation2d(xError, -yError);

    }

    public void zeroModules() {
        modules.forEach((m) -> {
            m.resetModulePositionToAbsolute();
        });
    }

    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    @Override
    public void update() {
        double timeStamp = Timer.getFPGATimestamp();
        drivingpose = Pose2d.fromRotation(getRobotHeading());
        Logger.getInstance().recordOutput("swerve state", currentState.name());

        switch (currentState) {
            case MANUAL:
                double rotationCorrection = headingController.updateRotationCorrection(drivingpose.getRotation(),
                        timeStamp);
                if (translationVector.norm() == 0 || rotationScalar != 0) {
                    rotationCorrection = 0;
                }
                SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
                commandModules(inverseKinematics.updateDriveVectors(translationVector,
                        rotationScalar + rotationCorrection, drivingpose, robotCentric));
                break;

            case SCORE:
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
                commandModules(inverseKinematics.updateDriveVectors(translationVector.translateBy(aimingVector),
                        rotationCorrection, drivingpose,
                        robotCentric));
                break;

            case BALANCE:
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(targetHeading, timeStamp);
                commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, drivingpose,
                        robotCentric));
                break;

            case TRAJECTORY:
                updateTrajectory();
                Translation2d translationCorrection = updateFollowedTranslation2d(timeStamp).scale(1);
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;
                Logger.getInstance().recordOutput("targetHeading", targetHeading.getDegrees());
                commandModules(inverseKinematics.updateDriveVectors(translationCorrection, rotationCorrection, pose,
                        robotCentric));
                break;

            case OBJECT:
                commandModules(
                        inverseKinematics.updateDriveVectors(aimingVector, rotationScalar, drivingpose, robotCentric));
                break;

            case OFF:
                commandModules(inverseKinematics.updateDriveVectors(new Translation2d(), 0, drivingpose, robotCentric));
                break;

            case SNAP:
                headingController.setTargetHeading(targetHeading);
                rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timeStamp);
                desiredRotationScalar = rotationCorrection;
                Logger.getInstance().recordOutput("rot", rotationCorrection);
                commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, pose,
                        robotCentric));
                break;

        }

    }

    public boolean balance() {
        double x = 0;
        double scalar = .009;
        if (pigeon.getPitch() > 1) {
            x = -scalar * pigeon.getPitch();
        } else if (pigeon.getPitch() < -1) {
            x = -scalar * pigeon.getPitch();
        } else {
            x = 0;
        }
        translationVector = new Translation2d(x, 0);
        if (DriverStation.getAlliance() == Alliance.Blue) {
            targetHeading = new Rotation2d();
        } else {
            targetHeading = new Rotation2d(180);
        }
        return Math.abs(x) < .03;

    }

    public void snap(double r) {
        targetHeading = Rotation2d.fromDegrees(r);
        headingController.setTargetHeading(targetHeading);
        Logger.getInstance().recordOutput("targetHEadingSnap", r);

    }

    public void Aim(Translation2d aimingVector, double scalar) {
        currentState = State.OBJECT;
        this.aimingVector = aimingVector;
        this.rotationScalar = scalar;
        update();
    }
    public void Aim(Pose2d aimingVector) {
        currentState = State.SCORE;
        this.aimingVector = aimingVector.getTranslation();
        targetHeading = aimingVector.getRotation();

        
        headingController.setTargetHeading(targetHeading);
        update();
    }

    public void Aim(Translation2d aimingVector, Rotation2d rotation) {
        currentState = State.SCORE;// SETS HEADING TO 0or 180
        this.aimingVector = aimingVector;
        targetHeading = rotation;
        headingController.setTargetHeading(rotation);
        update();

    }

    public void updateOdometry(double timestamp) {// uses sent input to commad modules and correct for rotatinol drift

        lastUpdateTimestamp = timestamp;

    }

    public Pose2d getPose() {
        return pose;
    }

    public double getRotationalVelSIM() {

        return desiredRotationScalar;
    }

    public void fieldzeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2d.fromDegrees(-180));
        gyro.setAngle(-180);
    }

    public double getAverageVel() {
        double average = 0;
        for (int i = 0; i < modules.size(); i++) {
            average = average + modules.get(i).currentvel();
        }
        return average / 4;
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

    public void resetGryo(double angle) {
        headingController.setTargetHeading(Rotation2d.fromDegrees(angle));
        gyro.setAngle(angle);
    }

    public void zeroSwerve() {// zeros gyro
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        gyro.setAngle(0);
        resetPose(new Pose2d(new Translation2d(1.9, 4.52), Rotation2d.fromDegrees(0)));
    }

    public void resetEncoders() {// zeros encoders
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).resetEncoders();
        }

    }

    public Request balanceRequest() {
        return new Request() {
            @Override
            public void act() {
                setState(State.BALANCE);
                balance();
            }

            @Override
            public boolean isFinished() {
                // if (balance())
                // setState(State.MANUAL);
                // return balance();
                return false;
            }
        };
    }

    public Request setStateRequest(State state) {
        return new Request() {
            @Override
            public void act() {
                setState(state);
            }
        };
    }

    public Request objectTargetRequest(boolean fixedRotation) {
        return new Request() {

            @Override
            public void act() {
                setState(State.SCORE);
                Aim(targetObject(fixedRotation));
            }

            @Override
            public void initialize() {
                Vision.getInstance().setPipeline(0);
            }

            @Override
            public boolean isFinished() {
                Translation2d translation = targetObject(fixedRotation).getTranslation();
                if( Math.abs(translation.getX())<.02 && Math.abs(translation.getY())<.04 && Vision.getInstance().hasTarget()){
                    aimingVector = new Translation2d();
                }
                return Math.abs(translation.getX())<.02 && Math.abs(translation.getY())<.04 && Vision.getInstance().hasTarget();
            }

        };
    }

    public Request goToNodeRequest(int node) {
        return new Request() {
            @Override
            public void act() {
                Aim(targetNode(false));
            }

            @Override
            public boolean isFinished() {
                if (aimFinished)
                    resetOffset();
                return aimFinished;
            }
        };

    }

    public Request goToChuteRequest() {

        return new Request() {
            @Override
            public void act() {
                gotoChute();
            }

            @Override
            public void initialize() {
                aimFinished = false;
            }

            @Override
            public boolean isFinished() {
                if (aimFinished)
                    resetOffset();
                return aimFinished;
            }
        };

    }

    public Request aimStateRequest(boolean cube) {
        return new Request() {

            public void act() {
                setState(State.SCORE);
                Aim(targetNode(cube));
            }

            @Override
            public void initialize() {
                Vision.getInstance().setPipeline(1);
            }

            @Override
            public boolean isFinished() {
                Translation2d translation = targetNode(cube).getTranslation();
                if( Math.abs(translation.getX())<.05 && Math.abs(translation.getY())<.05 && Vision.getInstance().hasTarget()){
                    aimingVector = new Translation2d();
                }
                return Math.abs(translation.getX())<.05 && Math.abs(translation.getY())<.05 && Vision.getInstance().hasTarget();
            }
        };

    }

    public void gotoChute() {
        double rotation = -90;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            rotation = 90;
        }
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - chuteLastTimeStamp;
        double Robotx = getPose().getTranslation().getX();
        xOPID.setSetpoint(2);
        double xError = xOPID.calculate(Robotx, dt);
        Aim(new Translation2d(-xError, 0), Rotation2d.fromDegrees(rotation));
        if (Math.abs(xError) < .03 && Math.abs(yError) < .3)
            aimTimer.start();
        else
            aimTimer.reset();
        if (aimTimer.get() > .5) {
            aimFinished = true;
        }
        chuteLastTimeStamp = currentTime;

    }

    public Pose2d targetNode(boolean cube) {

        double rotation = DriverStation.getAlliance() == Alliance.Blue ? 180 : 1;
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTimeStamp;
        if (Vision.getInstance().hasTarget()) {
            xVPID.setOutputRange(-.2, .2);
            yVPID.setOutputRange(-.2, .2);

            xError = xVPID.calculate(Vision.getInstance().getX(), dt);
            yError = yVPID.calculate(Vision.getInstance().getDistance()-.05, dt);
        }

        else {
            xError = 0;
            yError = 0;
        }

        Logger.getInstance().recordOutput("xError", xError);
        Logger.getInstance().recordOutput("yError", yError);
        lastTimeStamp = currentTime;
        return new Pose2d(new Translation2d(yError, xError).inverse(), Rotation2d.fromDegrees(180));
    }

    public Pose2d targetObject(boolean fixedRotation) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTimeStamp;
        if (Vision.getInstance().hasTarget()) {
            xVPID.setOutputRange(-.2, .2);
            aVPID.setOutputRange(-.2, .2);
            xVPID.setOutputMagnitude(.03);
            aVPID.setOutputMagnitude(.02
            );

            xError = xVPID.calculate(Vision.getInstance().getX(), dt);
            yError = aVPID.calculate(Vision.getInstance().getArea() - 20, dt);
        }

        else {
            xError = 0;
            yError = 0;
        }

        Logger.getInstance().recordOutput("xError", xError);
        Logger.getInstance().recordOutput("yError", yError);
        lastTimeStamp = currentTime;
        return new Pose2d(new Translation2d(yError, -xError).inverse(), (fixedRotation ? new Rotation2d() : getRobotHeading()));
    }

    public void updateOffset(boolean snapUp, boolean snapDown) {
        if (snapDown && (bestScore + offset + 1 < Constants.scoresY.size())) {
            aimTimer.reset();
            // if wants to move up and isnt at 10 than
            // move up
            offset++;// sets desired scoring station to snap to the next one up
        } else if (snapUp && (bestScore + offset - 1 >= 0)) {
            aimTimer.reset();
            // if wants to move down and isnt at zero than move down
            offset--;// sets desired scoring station to snap to the next one down
        }
    }

    public boolean isAiming() {
        return !aimFinished;
    }

    public Request openLoopRequest(Translation2d x, double r) {
        return new Request() {

            @Override
            public void act() {
                setState(State.MANUAL);
                sendInput(x.getX(), x.getY(), r);

            }

        };

    }

    public Request snapRequest(double r) {
        return new Request() {

            @Override
            public void act() {
                setState(State.SNAP);
                snap(r);
            }

            @Override
            public boolean isFinished() {
                return Math
                        .abs(headingController.getRotationCorrection(getRobotHeading(), Timer.getFPGATimestamp())) < .1;
            }

        };

    }

    public Request startPathRequest(double speed, boolean useAllianceColor) {
        return new Request() {
            @Override
            public void act() {
                setState(State.TRAJECTORY);
                startPath(speed, useAllianceColor);
            }

            @Override
            public boolean isFinished() {
                return trajectoryFinished;
            }
        };
    }

    public Request waitForTrajectoryRequest(double PercentageToRun) {
        return new Request() {
            @Override
            public boolean isFinished() {
                return pathFollower.hasElapsedPercentage(PercentageToRun);
            }
        };
    }

    public Request generateTrajectoryRequest(int node) {
        return new Request() {

            @Override
            public void act() {
                PathPlannerTrajectory trajectory = PathGenerator.generatePath(new PathConstraints(4, 4),
                        new Node(Constants.scoresY.get(node), DriverStation.getAlliance() == Alliance.Blue ? 2 : 14.71),
                        Constants.FieldConstants.obstacles);
                setTrajectory(trajectory);
            }

        };

    }

    public Request generateTrajectoryRequest(Node node) {
        return new Request() {

            @Override
            public void act() {
                PathPlannerTrajectory trajectory = PathGenerator.generatePath(new PathConstraints(4, 4), node,
                        Constants.FieldConstants.obstacles);
                setTrajectory(trajectory);
            }

        };

    }

    public Request setTrajectoryRequest(PathPlannerTrajectory trajectory, double x) {
        return new Request() {

            @Override
            public void act() {
                percentToRotate = x;
                setTrajectory(trajectory);
            }

        };
    }

    public void resetOffset() {
        offset = 0;
        ran = false;
        aimTimer.reset();
        aimTimer.stop();
    }

    public boolean goToObject(boolean cube) {
        Rotation2d heading = Rotation2d.fromDegrees(pigeon.getAngle());
        if (cube) {
            // vision.setPipeline(Constants.VisionConstants.CUBE_PIPELINE);
        } else {
            // vision.setPipeline(Constants.VisionConstants.CONE_PIPELNE);
        }
        double xSetPoint = (.1 * heading.getCos());
        double ySetPoint = (.1 * heading.getSin());
        double xError = xOPID.calculate(1 * heading.getCos(), xSetPoint);
        double yError = yOPID.calculate(1 * heading.getSin(), ySetPoint);
        double thetaControl = rPID.calculate(1, 0);

        Aim(new Translation2d(xError, yError), thetaControl);
        return true;
    }

    @Override
    public void outputTelemetry() {// outputs telemetry
        modules.forEach((m) -> {
            m.outputTelemetry();
        });
        Logger.getInstance().recordOutput("Odometry", pose.toWPI());
        Logger.getInstance().recordOutput("heading", getRobotHeading().getDegrees());
        Logger.getInstance().recordOutput("scoreError", errorFromScore);
        Logger.getInstance().recordOutput("targetHeading", targetHeading.getDegrees());
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
