// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.wcp.frc.Constants;
import com.wcp.frc.Options;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.encoders.Encoder;
import com.wcp.frc.subsystems.encoders.MagEncoder;
import com.wcp.lib.Conversions;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.Util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class SwerveDriveModule extends Subsystem {
    FlywheelSim rotationSim;
    FlywheelSim driveSim;
    TalonFX rotationMotor, driveMotor;
    Encoder rotationMagEncoder;
    String name;
    protected int moduleID;
    double encoderOffset;

    private double previousEncDistance = 0;
	private Translation2d position = new Translation2d();
    private Translation2d mstartingPosition;

	private Pose2d estimatedRobotPose = new Pose2d();
	boolean standardCarpetDirection = true;
    
    Translation2d modulePosition;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV,
            Constants.driveKA);

    boolean rotationEncoderFlipped;

    PeriodicIO mPeriodicIO = new PeriodicIO();

    /**
     * 
     * @param rotationMotorPort  -The Drive Motor Port
     * @param driveMotorPort     -The Drive Motor Port
     * @param moduleID           -The ID of the module
     * @param encoderStartingPos -The starting encoder position(used for zeroing
     *                           purposes)
     * @param modulePoseInches         -The position of the module relative to the robot's
     *                           center
     * @param flipEncoder        -Is the encoder going in the right direction?
     *                           (clockwise = increasing, counter-clockwise =
     *                           decreasing)
     */
    public SwerveDriveModule(int rotationMotorPort, int driveMotorPort, int moduleID, double encoderStartingPos,
            Translation2d modulePoseInches, boolean flipEncoder ,Translation2d moduleposemeters){
        this.rotationMotor = new TalonFX(rotationMotorPort);
        this.driveMotor = new TalonFX(driveMotorPort);
        this.driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 7.63, .3);
        this.moduleID = moduleID;
        this.name = "Module " + moduleID;
        this.encoderOffset = encoderStartingPos;
        this.modulePosition = modulePoseInches;
        this.mstartingPosition =moduleposemeters;


        this.rotationEncoderFlipped = flipEncoder;

        if (Options.encoderType == "Mag Encoder") {
            rotationMagEncoder = new MagEncoder(Ports.SWERVE_ENCODERS[moduleID]);
        } /*
           * else if(Options.encoderType == "Inductive Encoder") {
           * rotationMagEncoder = new InductiveEncoder(Ports.SWERVE_ENCODERS[moduleID]);
           * } else if(Options.encoderType == "CANCoder") {
           * rotationMagEncoder = new CANEncoder(Ports.SWERVE_ENCODERS[moduleID]);
           * }
           */

        configMotors();
    }

    public void configMotors() {
        rotationMotor.configFactoryDefault();
        driveMotor.configFactoryDefault();

        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        rotationMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kCANTimeoutMs);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.configVoltageCompSaturation(7.0, Constants.kCANTimeoutMs);
        rotationMotor.enableVoltageCompensation(true);
        rotationMotor.configAllowableClosedloopError(0, 0, Constants.kCANTimeoutMs);
        rotationMotor.configMotionAcceleration((int) (Constants.kSwerveRotationMaxSpeed * 15.5), 10);
        rotationMotor.configMotionCruiseVelocity((int) (Constants.kSwerveRotationMaxSpeed), 10);

        rotationMotor.selectProfileSlot(0, 0);
        // Slot 1 is for normal use
        rotationMotor.config_kP(0, 1, 10); // 1.55
        rotationMotor.config_kI(0, 0.0, 10);
        rotationMotor.config_kD(0, 5.0, 10); // 5.0
        rotationMotor.config_kF(0, 1023.0 / Constants.kSwerveRotationMaxSpeed, 10);
        rotationMotor.set(ControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));

        driveMotor.configFactoryDefault();
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        driveMotor.setSelectedSensorPosition(0, 0, 10);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);
        driveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms, 10);
        driveMotor.configVelocityMeasurementWindow(32, 10);
        driveMotor.configNominalOutputForward(0 / 12.0, 10);
        driveMotor.configNominalOutputReverse(0 / 12.0, 10);
        driveMotor.configVoltageCompSaturation(12.0, 10);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configOpenloopRamp(0.5, 10);
        driveMotor.configClosedloopRamp(0.0);
        driveMotor.configAllowableClosedloopError(0, 0, 10);

        driveMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.selectProfileSlot(0, 0);
        driveMotor.config_kP(0, 0.18, 10);
        driveMotor.config_kI(0, 0.0, 10);
        driveMotor.config_kD(0, 3.6, 10);
        driveMotor.config_kF(0, 1023.0 / Constants.kSwerveDriveMaxSpeed, 10);
        driveMotor.config_kP(1, 0, 10);
        driveMotor.config_kI(1, 0.0, 10);
        driveMotor.config_kD(1, 0, 10);
        driveMotor.config_kF(1, 1023.0 / Constants.kSwerveDriveMaxSpeed, 10);
        driveMotor.configMotionCruiseVelocity((int) (Constants.kSwerveDriveMaxSpeed * 0.9), 10);
        driveMotor.configMotionAcceleration((int) (Constants.kSwerveDriveMaxSpeed), 10);

    }

    public void invertDriveMotor(TalonFXInvertType invertType) {
        driveMotor.setInverted(invertType);
    }

    public void invertDriveMotor(boolean invert) {
        driveMotor.setInverted(invert);
    }

    public void invertRotationMotor(TalonFXInvertType invertType) {
        rotationMotor.setInverted(invertType);
    }

    public void invertRotationMotor(boolean invert) {
        rotationMotor.setInverted(invert);
    }

    public void setDriveMotorNeutralMode(NeutralMode mode) {
        driveMotor.setNeutralMode(mode);
    }

    public void setModuleAngle(double desiredAngle) {
        // SmartDashboard.putNumber(this.name + " Module Commanded Angle", desiredAngle);// output telemetry
        desiredAngle = Util.placeInAppropriate0To360Scope(getModuleAngle(), desiredAngle);// optimization
        double angleEncUnits = degreesToEncUnits(desiredAngle);
        mPeriodicIO.rotationDemand = angleEncUnits;
        mPeriodicIO.rotationControlMode = ControlMode.MotionMagic;
    }

    public double getModuleAngle() {
        return encUnitsToDegrees(mPeriodicIO.rotationPosition);
    }

    public double getModuleAbsolutePosition() {
        return rotationMagEncoder.getOutput() * ((this.rotationEncoderFlipped) ? -1 : 1) * 360.0;
    }

    public boolean isMagEncoderConnected() {
        return rotationMagEncoder.isConnected();
    }

    public void setDriveOpenLoop(double percentOuput) {// drives drive motor of pecentage
        driveMotor.selectProfileSlot(0, 0);
        mPeriodicIO.driveDemand = percentOuput;
        mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);

    }
    public synchronized void resetPose(Pose2d robotPose){
		Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(mstartingPosition)).getTranslation();
		position = modulePosition;
	}

    public Pose2d getEstimatedRobotPose(){
		return estimatedRobotPose;
	}

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading){
		Rotation2d normalizedAngle =Rotation2d.fromDegrees( getModuleAngle());
		return normalizedAngle.rotateBy(robotHeading);
	}
	
    public double encUnitsToInches(double encUnits){
		return encUnits/Constants.kSwerveEncUnitsPerInch;
	}
    public SwerveModulePosition getPosition() {// returns swerve module postition
        return new SwerveModulePosition(
                Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.kWheelCircumference, Options.driveRatio),
                Rotation2d.fromDegrees(getModuleAngle()));

    }
    public synchronized void updatePose(Rotation2d robotHeading){
		double currentEncDistance = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.kWheelCircumference, Options.driveRatio);
		double deltaEncDistance = (currentEncDistance - previousEncDistance) * Constants.kWheelScrubFactors[moduleID];
		Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
		Translation2d deltaPosition = new Translation2d(-currentWheelAngle.cos()*deltaEncDistance, 
				currentWheelAngle.sin()*deltaEncDistance);


		double xScrubFactor = Constants.kXScrubFactor;
		double yScrubFactor = Constants.kYScrubFactor;
        if(Util.epsilonEquals(Math.signum(deltaPosition.getX()), 1.0)){
            xScrubFactor = 1;

        }else{
            xScrubFactor = 1;
        }
        if(Util.epsilonEquals(Math.signum(deltaPosition.getY()), 1.0)){
            yScrubFactor = 1;
        }else{
            yScrubFactor = 1;
        }
	
		deltaPosition = new Translation2d(deltaPosition.getX() * xScrubFactor,
			deltaPosition.getY() * yScrubFactor);
        Logger.getInstance().recordOutput("delta t" + moduleID, deltaPosition.getY());
		Translation2d updatedPosition = position.translateBy(deltaPosition);
		Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
		Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(mstartingPosition).inverse());
		position = updatedPosition;
        Logger.getInstance().recordOutput("Pos " + moduleID,robotPose.toWPI());
		estimatedRobotPose =  robotPose;
		previousEncDistance = currentEncDistance;
	}

    public SwerveModuleState getState() {// gets states of module
        return new SwerveModuleState(
                Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Math.PI * 4, Options.driveRatio),
                Rotation2d.fromDegrees(getModuleAngle()));
    }

    public void setDriveDistanceMode(double distanceInches) {
        mPeriodicIO.driveControlMode = ControlMode.Position;
        mPeriodicIO.driveDemand = inchesToEncUnits(distanceInches);
    }

    public double currentvel(){
        return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.kWheelCircumference, Options.driveRatio);
    }

    private double degreesToEncUnits(double degrees) {
        return ((degrees / 360) * Constants.kSwerveRotationEncoderResolution) * Constants.kSwerveRotationReduction;
    }

    private double encUnitsToDegrees(double encUnits) {
        return ((encUnits / Constants.kSwerveRotationEncoderResolution) / Constants.kSwerveRotationReduction) * 360.0;
    }

    private double inchesToEncUnits(double inches) {
        return (inches / Constants.kOuterWheelDriveDiameter) * 2048.0 * Constants.kSwerveWheelReduction;
    }

    public void resetModulePositionToAbsolute() {
        if (rotationMotor.setSelectedSensorPosition(0) == ErrorCode.OK) {
            if (isMagEncoderConnected()) {
                double offset = getModuleAbsolutePosition() - encoderOffset;
                rotationMotor.setSelectedSensorPosition(degreesToEncUnits(offset), 0, 0);
            }
        } else {
            rotationMotorError = true;
        }
        if (driveMotor.setSelectedSensorPosition(0) != ErrorCode.OK)
            driveMotorError = true;
    }

    public enum ModuleStatus {
        OK, ABSOLUTE_ENCODER_ERROR, DRIVE_MOTOR_ERROR, ROTATION_MOTOR_ERROR;
    }

    private boolean rotationMotorError = false;
    private boolean driveMotorError = false;

    public ModuleStatus getModuleStatus() {
        if (!isMagEncoderConnected())
            return ModuleStatus.ABSOLUTE_ENCODER_ERROR;
        else if (driveMotorError)
            return ModuleStatus.DRIVE_MOTOR_ERROR;
        else if (rotationMotorError)
            return ModuleStatus.ROTATION_MOTOR_ERROR;
        return ModuleStatus.OK;
    }

    @Override
    public void writePeriodicOutputs() {// updates data
        mPeriodicIO.rotationPosition = rotationMotor.getSelectedSensorPosition();
        mPeriodicIO.drivePosition = driveMotor.getSelectedSensorPosition();
        mPeriodicIO.velocity = driveMotor.getSelectedSensorVelocity();
    }

    @Override
    public void readPeriodicInputs() {// uses data
        driveMotor.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
        rotationMotor.set(mPeriodicIO.rotationControlMode, mPeriodicIO.rotationDemand);
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput(this.name+" Angle Demand", encUnitsToDegrees(mPeriodicIO.rotationDemand));
        Logger.getInstance().recordOutput(this.name + " Angle", encUnitsToDegrees(mPeriodicIO.rotationPosition));
        Logger.getInstance().recordOutput(this.name + " Mag Encoder Raw Value", getModuleAbsolutePosition() / 360.0);
        Logger.getInstance().recordOutput(this.name + " Absolute Position", getModuleAbsolutePosition());
        Logger.getInstance().recordOutput(this.name + " Drive Motor Demand", mPeriodicIO.driveDemand);
        Logger.getInstance().recordOutput(this.name + " Status", getModuleStatus().toString());
    }

    @Override
    public void stop() {// stops everything
        setModuleAngle(getModuleAngle());
        setDriveOpenLoop(0.0);
    }

    public static class PeriodicIO {// data
        double rotationPosition = 0;
        double drivePosition = 0;
        double velocity = 0;

        ControlMode rotationControlMode = ControlMode.PercentOutput;
        ControlMode driveControlMode = ControlMode.PercentOutput;
        double rotationDemand = 0.0;
        double driveDemand = 0.0;
    }

}
