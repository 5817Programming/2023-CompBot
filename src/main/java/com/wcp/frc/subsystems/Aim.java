// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aim extends SubsystemBase {
  /** Creates a new Aim. */
  public Aim() {
    pigeon = Pigeon.getInstance();
    vision = Vision.getInstance();
  }


  PIDController rotationPID = new PIDController(0.029, 0, 0);// makes controllers
  PIDController yAxisPID = new PIDController(0.01, 0, 0);

  Vision vision;// defines types
  Gyro pigeon;
  double targetYaw;
  double gyroYaw;
  double range;
  double setPoint;
  Swerve swerve;

  @Override
  public void periodic() {

    setPoint = Constants.VisionConstants.OFFSETS.get(vision.setPoint);
    targetYaw = vision.getYaw();
    gyroYaw = pigeon.getAngle();
    range = vision.getDistance();
Logger.getInstance().recordOutput("gyro", gyroYaw);
    // This method will be called once per scheduler run
  }

  public void aim(double x) {

    swerve = Swerve.getInstance();

    double xPosition = swerve.getPose().getTranslation().x();
    double rotationCorrection = 0;
    boolean aimOptimize = false;
    double xCorrection = 0;
    double yCorrection = 0;
    
   if (range > 3.21) {
      xCorrection = (-(.39 * (range/12)) / 2)-.1;// p controller
    } 

    // if (translationVector.norm() <= .1) {
    // translationVector = new Translation2dd();
    // }

    if (gyroYaw < 0) {
      aimOptimize = false;
    } else {
      aimOptimize = true;
    }
    rotationCorrection = rotationPID.calculate(gyroYaw,180);
    if (vision.hasTarget()) {// goes towards target if passed charge station and has a tag
      yCorrection = yAxisPID.calculate(targetYaw, 0);

    }
    if (Math.abs(rotationCorrection) > .5) {
      rotationCorrection *= .5;
    }
// && range<7
    // } else {
    // if (r != 0){
    // rotationcorrection = r + rotationPID.calculate(targetyaw, setPoint);
    // }
    // }
    swerve.sendInput(x, yCorrection, rotationCorrection);// sends new inputs

  }
}
