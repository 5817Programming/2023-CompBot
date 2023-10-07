// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;



import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Requests.RequestList;

import edu.wpi.first.wpilibj.Timer;

public class Intake extends Subsystem {
  /** Creates a new Intake. */
  public Intake() {
    configMotor();
  }

  public void configMotor(){
      intake.configFactoryDefault();

        intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        intake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kCANTimeoutMs);
        intake.setNeutralMode(NeutralMode.Brake);
        intake.configVoltageCompSaturation(7.0, Constants.kCANTimeoutMs);
        intake.enableVoltageCompensation(true);
        intake.configAllowableClosedloopError(0, 0, Constants.kCANTimeoutMs);
        intake.configMotionAcceleration((int) (Constants.kSwerveRotationMaxSpeed * 15.5), 10);
        intake.configMotionCruiseVelocity((int) (Constants.kSwerveRotationMaxSpeed), 10);

        intake.selectProfileSlot(0, 0);
        // Slot 1 is for normal use
        intake.config_kP(0, 0.02, 10); // 1.55
        intake.config_kI(0, 0.0, 10);
        intake.config_kD(0, 1.0, 10); // 5.0
        intake.config_kF(0, 200.0 / Constants.kSwerveRotationMaxSpeed, 10);
        intake.set(ControlMode.MotionMagic, intake.getSelectedSensorPosition(0));

 
  }

  public static Intake instance = null;

  public static Intake getInstance(){
        if(instance == null)
          instance = new Intake();
      return instance;
  }

  TalonFX intake = new TalonFX(Ports.intake);
  boolean isIntaking = false;
  Timer waitTimer = new Timer();

  PeriodicIO mPeriodicIO = new PeriodicIO();

  public void intake(double percent,boolean reversed){
    mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
    mPeriodicIO.driveDemand = reversed ? -percent: percent;

  
}

public Request percentOutputRequest(double percent, boolean cube){
    return new Request() {
      @Override
        public void act(){
          waitTimer.start();
          intake(percent,cube);
        }
      @Override
        public boolean isFinished(){
            if(waitTimer.hasElapsed(.4)){
              waitTimer.reset();
              waitTimer.stop();
            }
          return waitTimer.hasElapsed(.4);
        }
    };
}
public Request percentOutputRequest(boolean cube){
  return new Request() {
    @Override
      public void act(){
        waitTimer.start();
        intake(cube ? 1: .5, !cube);
      }
      @Override
      public boolean isFinished(){
          if(waitTimer.hasElapsed(.4)){
            waitTimer.reset();
            waitTimer.stop();
            return true;
          }
          else return false;
      }
  };
}

public Request continuousIntakeRequest(boolean cube){
  return new Request(){
        @Override
      public void act(){
           intake(cube ? 1: 1, !cube);
           isIntaking = true;
      }
    
  };
  }

  public Request setPercentRequest(double precent){
    return new Request(){
      @Override
      public void act() {
        setPercentOutput(precent);
      }};


  }




public Request stopIntakeRequest(){
  return new Request(){
    @Override
      public void act(){
        intake(0, false);
      }
  };
}

public Request waitUntilIntakedPieceRequest(){
  return new Request(){
      public void initialize(){
        waitTimer.reset();
        waitTimer.start();
      }
    @Override
      public void act(){

      }
      
    @Override
      public boolean isFinished(){
        return intake.getStatorCurrent()>60 || waitTimer.get() > 2;
      }
  };
}

public Request intakeBrakeRequest(){
  return new Request(){
    @Override
      public void act(){
          brakeIntake();
          isIntaking = false;
      }
  };
}

public void brakeIntake(){
  intake.selectProfileSlot(0,0);
  mPeriodicIO.driveControlMode = ControlMode.MotionMagic;
  mPeriodicIO.driveDemand = mPeriodicIO.drivePosition;
}

public Request percentOutputRequest(double percent, boolean cube,double waitTime){
  return new Request() {
    @Override
      public void act(){
        waitTimer.start();
        intake(percent,cube);
      }
    @Override
      public boolean isFinished(){
          if(waitTimer.hasElapsed(waitTime)){
            waitTimer.reset();
            waitTimer.stop();
          }
        return waitTimer.hasElapsed(waitTime);
      }
  };
}

  public Request percentOutputRequest(boolean cube, double waitTime){
    return new Request() {
      @Override
        public void act(){
          waitTimer.start();
          intake(cube ? .3 : 1, cube);
        }
        @Override
        public boolean isFinished(){
            if(waitTimer.hasElapsed(waitTime)){
              waitTimer.reset();
              waitTimer.stop();
            }
          return waitTimer.hasElapsed(waitTime);
        }
    };



}

public boolean isIntaking(){
  return isIntaking;
}

public void setPercentOutput(double cube){
  mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
  mPeriodicIO.driveDemand = cube;
}

@Override
public void stop() {  
  setPercentOutput(0);
}
@Override
public void writePeriodicOutputs() {
	mPeriodicIO.drivePosition = intake.getSelectedSensorPosition();
	mPeriodicIO.velocity = intake.getSelectedSensorVelocity();
}
@Override
public void readPeriodicInputs() {
	intake.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
}
  

  public static class PeriodicIO  {
	double drivePosition = 0;
	double velocity = 0;

	ControlMode driveControlMode = ControlMode.MotionMagic;
	double rotationDemand = 0.0;
	double driveDemand = 0.0;
}
  
  @Override
  public void outputTelemetry() {
    Logger.getInstance().recordOutput("intakeDemand", mPeriodicIO.driveDemand);		
    Logger.getInstance().recordOutput("Timwr",waitTimer.get());


  }
}

