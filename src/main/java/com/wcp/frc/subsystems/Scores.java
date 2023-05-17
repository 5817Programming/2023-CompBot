// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scores extends SubsystemBase {
  public double armOffset = 0;
  public double sideElvator = 0;

  public double elevatorOffset =  0;
  public boolean Cube= true;
  

  /** Creates a new Scores. */
  public  Scores() {

  }

  @Override
  public void periodic() {
    if (height == Constants.ElevatorConstants.LOW_CONE&&Cube) {
      scoreLowCube();

    }
    if (height == Constants.ElevatorConstants.MID_CONE&&Cube) {
      scoreMidCube();
     
    }
    if (height == Constants.ElevatorConstants.HIGH_CONE&&Cube) {
      scoreHighCube();
    }
    if (height == Constants.ElevatorConstants.HOOMAN_CONE&&Cube ) {
      HOOMAN_CUBE();
    }
    
    if (height == Constants.ElevatorConstants.LOW_CONE&&Cube == false) {
      scoreLowCone();

    }
    if (height == Constants.ElevatorConstants.MID_CONE&&Cube == false) {
      scoreMidCone();
     
    }
    if (height == Constants.ElevatorConstants.HIGH_CONE&&Cube == false) {
      scoreHighCone();
    }
    if (height == Constants.ElevatorConstants.HOOMAN_CONE&&Cube == false) {
      hooman_CONE();
    }
    if (height == Constants.ElevatorConstants.ZERO) {
      zero();
    }
    if (height == Constants.ElevatorConstants.HOLD) {
      hold();
    }

    if (height == Constants.ElevatorConstants.PICKUP) {
      pickup();
    }
    // This method will be called once per scheduler run
  }

  // Vision vision;
  Elevator elevator = new Elevator();
  SideElevator sideElevator = new SideElevator();

  Arm arm = new Arm();

  double height = Constants.zero;
  Constants constants;
  public void setHeight(double height, boolean Cube){
    this.height = height;
    this.Cube = Cube;
    Logger.getInstance().recordOutput("heighst", height);

  }

  public void score(boolean coDriverA, boolean coDriverB, boolean coDriverX, boolean coDriverY, 
  boolean coDriverBackButton, boolean coDriverLeftStickDown, boolean coDriverLeftBumperTAP, boolean Cube) {
    
    if (coDriverA) {
      height = Constants.ElevatorConstants.LOW_CONE;
    } else if (coDriverB) {
      height = Constants.ElevatorConstants.MID_CONE;
    } else if (coDriverX) {
      height = Constants.ElevatorConstants.HOOMAN_CONE;
    } else if (coDriverY) {
      height = Constants.ElevatorConstants.HIGH_CONE;
    } else if (coDriverBackButton) {
      height = Constants.ElevatorConstants.ZERO;
    } else if (coDriverLeftStickDown) {
      height = Constants.ElevatorConstants.HOLD;
    } else if (coDriverLeftBumperTAP) {

    }
    this.Cube = Cube;

  }

  public void scoring() {

    if (height == Constants.ElevatorConstants.LOW_CONE&&Cube) {
      scoreLowCube();

    }
    if (height == Constants.ElevatorConstants.MID_CONE&&Cube) {
      scoreMidCube();
     
    }
    if (height == Constants.ElevatorConstants.HIGH_CONE&&Cube) {
      scoreHighCube();
    }
    if (height == Constants.ElevatorConstants.HOOMAN_CONE&&Cube ) {
      HOOMAN_CUBE();
    }
    
    if (height == Constants.ElevatorConstants.LOW_CONE&&Cube == false) {
      scoreLowCone();

    }
    if (height == Constants.ElevatorConstants.MID_CONE&&Cube == false) {
      scoreMidCone();
     
    }
    if (height == Constants.ElevatorConstants.HIGH_CONE&&Cube == false) {
      scoreHighCone();
    }
    if (height == Constants.ElevatorConstants.HOOMAN_CONE&&Cube == false) {
      hooman_CONE();
    }
    if (height == Constants.ElevatorConstants.ZERO) {
      zero();
    }
    if (height == Constants.ElevatorConstants.HOLD) {
      hold();
    }

    if (height == Constants.ElevatorConstants.PICKUP) {
      pickup();
    }
  }

  public void hold() {
    sideElevator.elevator(0+sideElvator);
    elevator.elevator(Constants.ElevatorConstants.MAX_DOWN);
    arm.setMotionMagic(Constants.ArmConstants.HOLD+ armOffset);
     armOffset=0;
     sideElvator=0;
  }

  public void hooman_CONE() {
    sideElevator.elevator(Constants.SideElevatorConstants.HOOMAN+sideElvator);// go to set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.HOOMAN_CONE);
      arm.setMotionMagic(Constants.ArmConstants.HOOMAN+armOffset);
      
   

  }
  public void HOOMAN_CUBE() {
    sideElevator.elevator(Constants.SideElevatorConstants.HOOMAN+sideElvator);// go to set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.HOOMAN_CUBE);
      arm.setMotionMagic(Constants.ArmConstants.HOOMAN+armOffset);
      
   

  }

  public void scoreLowCone() {
    sideElevator.elevator(Constants.SideElevatorConstants.MAX_DOWN+sideElvator);// go to set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.LOW_CONE);
    arm.setMotionMagic(Constants.ArmConstants.LOW_SCORE_CONE + armOffset);

  }
 

  public void scoreMidCone() {
    sideElevator.elevator(Constants.SideElevatorConstants.MAX_DOWN+sideElvator);// go to set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.MID_CONE);
    arm.setMotionMagic(Constants.ArmConstants.MIDDLE_CONE + armOffset);

  }

  // (((vision.range)/12)-3)*(2048/Constants.sideelevatorgearratio)*Constants.sideelevatorrotationsperfoot
  public void scoreHighCone() {
    sideElevator.elevator(Constants.SideElevatorConstants.HIGH_CONE+sideElvator);// goto set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.HIGH_CONE);

    arm.setMotionMagic(Constants.ArmConstants.HIGH_CONE + armOffset);

  }
  public void scoreLowCube() {
    sideElevator.elevator(Constants.SideElevatorConstants.MAX_DOWN+sideElvator);// go to set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.LOW_CUBE);
    arm.setMotionMagic(Constants.ArmConstants.LOW_SCORE_CUBE + armOffset);

  }
 

  public void scoreMidCube() {
    sideElevator.elevator(Constants.SideElevatorConstants.MAX_DOWN+sideElvator);// go to set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.MID_CUBE);
    arm.setMotionMagic(Constants.ArmConstants.MIDDLE_CUBE + armOffset);

  }

  // (((vision.range)/12)-3)*(2048/Constants.sideelevatorgearratio)*Constants.sideelevatorrotationsperfoot
  public void scoreHighCube() {
    sideElevator.elevator(Constants.SideElevatorConstants.HIGH_CUBE+sideElvator);// goto set offset based off vision
    elevator.elevator(Constants.ElevatorConstants.HIGH_CUBE);
    arm.setMotionMagic(Constants.ArmConstants.HIGH_CUBE + armOffset);

  }

  public void pickup() {
    sideElevator.elevator(Constants.SideElevatorConstants.PICKUP+sideElvator);
    elevator.elevator(Constants.ElevatorConstants.PICKUP - elevatorOffset);
    arm.setMotionMagic(Constants.ArmConstants.PICKUP);

  }

  public void zero() {
    sideElevator.elevator(Constants.SideElevatorConstants.ZERO+sideElvator);
    elevator.elevator(Constants.ElevatorConstants.ZERO);
    arm.setMotionMagic(Constants.ArmConstants.ZERO+500);
         armOffset=0;
         sideElvator = 0;
  }
}
