// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;


public class SideElevator extends Subsystem {
	

	public static SideElevator instance = null;

	public static SideElevator getInstance(){
		if(instance == null)
			instance = new SideElevator();
		return instance;
	}
	PeriodicIO mPeriodicIO = new PeriodicIO();

	/* Setpoints */
	double mTargetMin = 1000;//500
	double mTargetMax = 260000;//78000
	double targetPos;

	/* Hardware */
	TalonFX side = new TalonFX(Ports.kSideElevatorPort);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	
	public enum State{
		HIGHCONE(Constants.SideElevatorConstants.HIGH_CONE),
		HIGHCUBE(Constants.SideElevatorConstants.HIGH_CUBE),
		MIDCONE(Constants.SideElevatorConstants.MID_CONE),
		MIDCUBE(Constants.SideElevatorConstants.MID_CUBE),
		LOWCONE(Constants.SideElevatorConstants.LOW_CONE),
		LOWCUBE(Constants.SideElevatorConstants.LOW_CUBE),
		ZERO(Constants.SideElevatorConstants.ZERO),
		CHUTE(Constants.SideElevatorConstants.HOLD),
		HOOMANCONE(Constants.SideElevatorConstants.HOOMAN),
		HOOMANCUBE(Constants.SideElevatorConstants.HOOMAN),
		PICKUP(Constants.SideElevatorConstants.ZERO);

		double output = 0;

		private State(double output){
            this.output = output;
        }
	}
	private State currentState = State.ZERO;

	public SideElevator() {
			/* Factory default hardware to prevent unexpected behavior */
			side.configFactoryDefault();
	
			/* Set to Brake Mode */
			side.setNeutralMode(NeutralMode.Brake);
	
			side.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
					Constants.TIMEOUT_MILLISECONDS);
	
			side.configNeutralDeadband(0.001, Constants.TIMEOUT_MILLISECONDS);

			side.setSensorPhase(false);
			side.setInverted(true);

			side.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT_MILLISECONDS);
			side.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TIMEOUT_MILLISECONDS);
	
			side.configNominalOutputForward(0, Constants.TIMEOUT_MILLISECONDS);
			side.configNominalOutputReverse(0, Constants.TIMEOUT_MILLISECONDS);
			side.configPeakOutputForward(1, Constants.TIMEOUT_MILLISECONDS);
			side.configPeakOutputReverse(-1, Constants.TIMEOUT_MILLISECONDS);
			
			side.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
			side.config_kF(Constants.kSlotIdx, 0.04731948749, Constants.TIMEOUT_MILLISECONDS);//0.0649
			side.config_kP(Constants.kSlotIdx, 0.05, Constants.TIMEOUT_MILLISECONDS);//0.7161
			side.config_kI(Constants.kSlotIdx, 0, Constants.TIMEOUT_MILLISECONDS);//0.001
			side.config_kD(Constants.kSlotIdx, 0.5, Constants.TIMEOUT_MILLISECONDS);//P value * ten

			side.configMotionCruiseVelocity(3619, Constants.TIMEOUT_MILLISECONDS);
			side.configMotionAcceleration(12476, Constants.TIMEOUT_MILLISECONDS);
	
			side.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.TIMEOUT_MILLISECONDS);
			
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
	};
}

public Request percenRequest(double percent){
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

  public void my_PercentOutput( double speed){
	mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
	mPeriodicIO.driveDemand = speed;  }

  public double getEncoder(){
	Logger.getInstance().recordOutput("side elevator", side.getSelectedSensorPosition());
	return side.getSelectedSensorPosition();
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
	mPeriodicIO.drivePosition = side.getSelectedSensorPosition();
	mPeriodicIO.velocity = side.getSelectedSensorVelocity();
}
@Override
public void readPeriodicInputs() {
	side.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
}
public static class PeriodicIO  {
	double drivePosition = 0;
	double velocity = 0;

	ControlMode driveControlMode = ControlMode.MotionMagic;
	double rotationDemand = 0.0;
	double driveDemand = 0.0;
}



}
