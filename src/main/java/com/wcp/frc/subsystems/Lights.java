// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.wcp.frc.Ports;
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

  public void setLights(boolean codriverRightBumper, boolean codriverLeftBumper) {// sets lights based off input
    // Vision.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    if (codriverRightBumper) {
        lights.set(Constants.LightConstants.CONE_LIGHT);
    }
      else if (codriverLeftBumper) {
        lights.set(Constants.LightConstants.CUBE_LIGHT);
       
      }
    }
    // vision.setPipelineIndex(Constants.ConePip);//if gets what you are looking for
    // set lights to normal
    // if (vision.hasTarget()){
    // setLED(Constants.normallight);//rainbow(-.99)
    // }else{
    // vision.setPipelineIndex(Constants.CubePip);
    // if (vision.hasTarget()){
    // setLED(Constants.normallight);//rainbow(-.99)
    // }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    
  }

  }

