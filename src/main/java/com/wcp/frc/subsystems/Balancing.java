// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;



public class Balancing extends SubsystemBase {

  public static Balancing instance = null;
    public static Balancing getInstance() {
        if(instance == null)
            instance = new Balancing();
        return instance;
    }
  
  Gyro pigeon;
  Swerve swerve;

  double scalar = .012;
  public boolean toggle;

  public Balancing() {
    pigeon = Pigeon.getInstance();
    swerve = Swerve.getInstance();
  }

  public double x = 0;
  double direction =1;
  

  public void balance(){

    if(toggle){
      if(pigeon.getPitch()>1){
        x = -scalar * pigeon.getPitch();
      }else if(pigeon.getPitch() < -1){
        x = -scalar * pigeon.getPitch();
      }else{
        x = 0;
      }
      swerve.sendInput(x, 0, 0);//gives new swerve instructions
    }
  
  }

  @Override
  public void periodic() {

    if(toggle){
      if(pigeon.getPitch()>1){
        x = -scalar * pigeon.getPitch();
      }else if(pigeon.getPitch() < -1){
        x = -scalar * pigeon.getPitch();
      }else{
        x = 0;
      }
      swerve.sendInput(x, 0, 0);//gives new swerve instructions
    }
    // This method will be called once per scheduler run
  }
}
