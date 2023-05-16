// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;


public class Elevator extends Subsystem {

	public static Elevator instance = null;

	public static Elevator getInstance(){
		if(instance == null)
			instance = new Elevator();
		return instance;
	}

	PeriodicIO mPeriodicIO = new PeriodicIO();
	/* Setpoints */
	double mTargetMin = 800;//500
	double mTargetMax = 78000;//78000
	double targetPos;

	/* Hardware */
	TalonFX mLeftElevator = new TalonFX(Ports.kLeftElevatorPort);
	TalonFX mRightElevator = new TalonFX(Ports.kRightElevatorPort);
	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	public enum State{
		HIGHCONE(Constants.ElevatorConstants.HIGH_CONE),
		HIGHCUBE(Constants.ElevatorConstants.HIGH_CUBE),
		MIDCONE(Constants.ElevatorConstants.MID_CONE),
		MIDCUBE(Constants.ElevatorConstants.MID_CUBE),
		LOWCONE(Constants.ElevatorConstants.LOW_CONE),
		LOWCUBE(Constants.ElevatorConstants.LOW_CUBE),
		ZERO(Constants.ElevatorConstants.ZERO),
		CHUTE(Constants.ElevatorConstants.HOLD),
		HOOMANCONE(Constants.ElevatorConstants.HOOMAN_CONE),
		HOOMANCUBE(Constants.ElevatorConstants.HOOMAN_CUBE),
		PICKUP(Constants.ElevatorConstants.ZERO);

		double output = 0;

		private State(double output){
            this.output = output;
        }
	}
	private State currentState = State.ZERO;
	public Elevator() {
			mLeftElevator.configFactoryDefault();
			mRightElevator.configFactoryDefault();

			mLeftElevator.setNeutralMode(NeutralMode.Brake);
			mRightElevator.setNeutralMode(NeutralMode.Brake);
			mRightElevator.follow(mLeftElevator);
			mRightElevator.setInverted(TalonFXInvertType.FollowMaster);
			mLeftElevator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
					Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.configNeutralDeadband(0.001, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.setSensorPhase(false);
			mLeftElevator.setInverted(false);
			mLeftElevator.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.configNominalOutputForward(0, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.configNominalOutputReverse(0, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.configPeakOutputForward(1, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.configPeakOutputReverse(-1, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
			mLeftElevator.config_kF(Constants.kSlotIdx, 0.05757217626, Constants.TIMEOUT_MILLISECONDS);//0.0649
			mLeftElevator.config_kP(Constants.kSlotIdx, 0.344444444, Constants.TIMEOUT_MILLISECONDS);//0.7161
			mLeftElevator.config_kI(Constants.kSlotIdx, 0.001, Constants.TIMEOUT_MILLISECONDS);//0.001
			mLeftElevator.config_kD(Constants.kSlotIdx, 3.4444444, Constants.TIMEOUT_MILLISECONDS);//P value * ten
			mLeftElevator.configMotionCruiseVelocity(16881, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.configMotionAcceleration(16880.55, Constants.TIMEOUT_MILLISECONDS);
			mLeftElevator.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.TIMEOUT_MILLISECONDS);
			
}
  

public void setState(State state){
	currentState = state;
}

public State getState(){
	return currentState;
}

public void conformToState(State newState){
	elevator(newState.output);
}

public Request stateRequest(State newState){
	return new Request() {
		@Override
			public void act() {
				conformToState(newState);
			}
		@Override
			public boolean isFinished(){
				return Math.abs(mPeriodicIO.driveDemand-mPeriodicIO.drivePosition)<1000;
			}
	};
}
public Request idleRequest(){
	return new Request() {
		@Override
			public void act() {
				conformToState(State.ZERO);
			}
	};
}

public Request percentRequest(double percent){
	return new Request() {
		@Override
			public void act() {
				my_PercentOutput(percent);
			}
	};
}


  public void elevator( double targetPos){
	mPeriodicIO.driveControlMode = ControlMode.MotionMagic;
	mPeriodicIO.driveDemand = targetPos;

  }

  public void my_PercentOutput(double speed){
	
	mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
	mPeriodicIO.driveDemand = speed;  
  }
  public double getEncoder(){
	Logger.getInstance().recordOutput("elevator", mLeftElevator.getSelectedSensorPosition());

	return mLeftElevator.getSelectedSensorPosition();
  }


@Override
public void outputTelemetry() {
	// TODO Auto-generated method stub
	
}


@Override
public void stop() {
	// TODO Auto-generated method stub
	
}
@Override
public void writePeriodicOutputs() {
	mPeriodicIO.drivePosition = mLeftElevator.getSelectedSensorPosition();
	mPeriodicIO.velocity = mLeftElevator.getSelectedSensorVelocity();
}
@Override
public void readPeriodicInputs() {
	mLeftElevator.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
}
public static class PeriodicIO  {
	double drivePosition = 0;
	double velocity = 0;

	ControlMode driveControlMode = ControlMode.MotionMagic;
	double rotationDemand = 0.0;
	double driveDemand = 0.0;
}
}
