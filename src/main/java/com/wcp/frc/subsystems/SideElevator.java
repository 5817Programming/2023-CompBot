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


public class SideElevator extends SubsystemBase {
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
	public SideElevator() {
			/* Factory default hardware to prevent unexpected behavior */
			side.configFactoryDefault();
	
			/* Set to Brake Mode */
			side.setNeutralMode(NeutralMode.Brake);
	
			/* Will follow so no need to config further */

			/* Configure Sensor Source for Pirmary PID */
			side.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
					Constants.TIMEOUT_MILLISECONDS);
	
			/* set deadband to super small 0.001 (0.1 %).
				The default deadband is 0.04 (4 %) */
			side.configNeutralDeadband(0.001, Constants.TIMEOUT_MILLISECONDS);
	
			/**
			 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
			 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
			 * sensor to have positive increment when driving Talon Forward (Green LED)
			 */

			side.setSensorPhase(false);
			side.setInverted(true);
	
			/*
			 * Talon FX does not need sensor phase set for its integrated sensor
			 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
			 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
			 * 
			 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
			 */
			// mLeftElevator.setSensorPhase(true);
	
			/* Set relevant frame periods to be at least as fast as periodic rate */
			side.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT_MILLISECONDS);
			side.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TIMEOUT_MILLISECONDS);
	
			/* Set the peak and nominal outputs */
			side.configNominalOutputForward(0, Constants.TIMEOUT_MILLISECONDS);
			side.configNominalOutputReverse(0, Constants.TIMEOUT_MILLISECONDS);
			side.configPeakOutputForward(1, Constants.TIMEOUT_MILLISECONDS);
			side.configPeakOutputReverse(-1, Constants.TIMEOUT_MILLISECONDS);
			
			/* Set Motion Magic gains in slot0 - see documentation */
			side.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
			side.config_kF(Constants.kSlotIdx, 0.04731948749, Constants.TIMEOUT_MILLISECONDS);//0.0649
			side.config_kP(Constants.kSlotIdx, 0.05, Constants.TIMEOUT_MILLISECONDS);//0.7161
			side.config_kI(Constants.kSlotIdx, 0, Constants.TIMEOUT_MILLISECONDS);//0.001
			side.config_kD(Constants.kSlotIdx, 0.5, Constants.TIMEOUT_MILLISECONDS);//P value * ten
	
			/*mLeftElevator.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
			mLeftElevator.config_kF(Constants.kSlotIdx, 0.05757217626, Constants.kTimeoutMs);//0.0649
			mLeftElevator.config_kP(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//0.7161
			mLeftElevator.config_kI(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//0.001
			mLeftElevator.config_kD(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//P value * ten
	*/
			/* Set acceleration and vcruise velocity - see documentation */
			side.configMotionCruiseVelocity(3619, Constants.TIMEOUT_MILLISECONDS);
			side.configMotionAcceleration(12476, Constants.TIMEOUT_MILLISECONDS);
	
			/* Zero the sensor once on robot boot up */
			side.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.TIMEOUT_MILLISECONDS);
			
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	
  }

  public void elevator( double targetPos){
    side.set(ControlMode.MotionMagic, targetPos);

  }

  public void my_PercentOutput( double speed){
   side.set(ControlMode.PercentOutput, speed);
  }

  public double getEncoder(){
	Logger.getInstance().recordOutput("side elevator", side.getSelectedSensorPosition());
	return side.getSelectedSensorPosition();
  }
  


}
