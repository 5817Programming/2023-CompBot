// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;


import java.security.DrbgParameters.Reseed;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.wcp.frc.Controls;
import com.wcp.frc.Ports;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
  TalonFX intake = new TalonFX(Ports.intake);
  
  boolean stop = false;
  double ramp;
  boolean hold= false;
  Scores scores = new Scores();
  boolean hasPiece= false;
  boolean isReversed;
  boolean intaked;
  
  public void intake(double percent,boolean reversed,boolean out){
    intaked=false;
    
    intake.setNeutralMode(NeutralMode.Brake);
    if(!out)
    // if the intake isn't at what where we want it it progressively gets faster until it is where it needs to be 
    {if(percent>ramp){
      ramp += 0.02;
    }
    intake.set(TalonFXControlMode.PercentOutput, percent);
    // if intake reversed and current is more than 22 then it stops if not reversed and current is more than 100 it stops
    if(intake.getSelectedSensorVelocity()<3000&&ramp>.3){
      stop = true;
      hasPiece = true;
    }
    if(stop){
      ramp = 0;
    } 
   intake.set(ControlMode.PercentOutput,(reversed?1:-1));
  }else{
    intake.set(ControlMode.PercentOutput,(reversed?-1:1));

  }
  Logger.getInstance().recordOutput("volatge", intake.getStatorCurrent());
  this.isReversed = reversed;
}
public void setIntake (){
  intaked= true;

}
public void outake (boolean cube,double cubespeed, double conespeed){
  if(.2>ramp){
    ramp = ramp +0.01;
  }
  if(ramp>.18)
  {
    stop = false;
    hold = true;
   }
  Logger.getInstance().recordOutput("ramp", ramp);
  if(!stop){
    hasPiece= false;
    intaked = false;
    intake.set(ControlMode.PercentOutput, cube ? cubespeed:conespeed);
  }

  
  
  
}
public void setPercentOutput(double p){
  intake.set(ControlMode.PercentOutput, -p);

}
  public void stop(){
    stop = false;
    hold = false;
    ramp = 0;
    intake.set(ControlMode.PercentOutput, 0);

  }
  @Override
  public void periodic() {
    if (hasPiece&intaked){
      if(intake.getSelectedSensorVelocity()<2000){
        intake.set(ControlMode.PercentOutput, isReversed ? .1:-.1);
     
      }
    }

  }
}
