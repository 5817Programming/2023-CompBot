// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.Requests.Request;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.wcp.frc.Constants;

public class Lights extends Subsystem {
  /** Creates a new Lights. */
  private static Lights instance_;

  public static Lights getInstance() {
    if (instance_ == null) {
      instance_ = new Lights();
    }
    return instance_;
  }

  public Lights() {
  }

  Ports ports = new Ports();
  // Vision vision;
  double color = Constants.LightConstants.NORMAL_LIGHT;

  Spark lights = new Spark(Ports.light);

  public enum State{
    CUBE(Constants.LightConstants.CUBE_LIGHT),
    CONE(Constants.LightConstants.CONE_LIGHT),
    AIMING(Constants.LightConstants.NORMAL_LIGHT),
    PICKUP(Constants.LightConstants.CONE_LIGHT),
    SCORING(Constants.LightConstants.CUBE_LIGHT);

    double output = 0;
    private State(double output){
      this.output = output;
    }
  }

  public void conformToState(State newState) {// sets lights based off input
        lights.set(newState.output);
    }

  
  public Request lighRequest(State newState){
      return new Request() {

        @Override
        public void act() {
          conformToState(newState);
        }
        
      };
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    
  }

  }

