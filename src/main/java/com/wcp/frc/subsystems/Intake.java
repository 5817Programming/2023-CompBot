// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;



import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        intake.config_kP(0, 1, 10); // 1.55
        intake.config_kI(0, 0.0, 10);
        intake.config_kD(0, 5.0, 10); // 5.0
        intake.config_kF(0, 1023.0 / Constants.kSwerveRotationMaxSpeed, 10);
        intake.set(ControlMode.MotionMagic, intake.getSelectedSensorPosition(0));

 
  }

  public static Intake instance = null;

  public static Intake getInstance(){
        if(instance == null)
          instance = new Intake();
      return instance;
  }

  TalonFX intake = new TalonFX(Ports.intake);
  
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
          waitTimer.reset();
          waitTimer.start();
          intake(percent,cube);
        }
      @Override
        public boolean isFinished(){
            if(waitTimer.hasElapsed(.2)){
              waitTimer.reset();
              waitTimer.stop();
            }
          return waitTimer.hasElapsed(.2);
        }
    };
}

public Request percentOutputRequest(boolean cube){
  return new Request() {
    @Override
      public void act(){
        waitTimer.reset();
        waitTimer.start();
        intake(cube ? .3 : 1, cube);
      }
      @Override
      public boolean isFinished(){
          if(waitTimer.hasElapsed(.2)){
            waitTimer.reset();
            waitTimer.stop();
          }
        return waitTimer.hasElapsed(.2);
      }
  };
}

public Request stopIntakeRequest(){
  return new Request(){
    @Override
      public void act(){
        intake(0, false);
      }
  }
}

public Request waitUntilIntakedPieceRequest(){
  return new Request(){
    @Override 
      public void act(){

      }
    @Override
      public boolean isFinished(){
        return intake.getStatorCurrent()>60;
      }
  }
}

public Request intakeBrakeRequest(){
  return new Request(){
    @Override
      public void act(){
          brakeIntake();
      }
  }
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
        waitTimer.reset();
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
          waitTimer.reset();
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

public void setPercentOutput(double p){
  mPeriodicIO.driveDemand = p;
  mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
}


  @Override
public void outputTelemetry() {
	// TODO Auto-generated method stub
	
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
    // TODO Auto-generated method stub
    
  }
}

