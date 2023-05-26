// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.ArrayList;
import java.util.List;

import com.wcp.frc.subsystems.Arm;
import com.wcp.frc.subsystems.Elevator;
import com.wcp.frc.subsystems.Intake;
import com.wcp.frc.subsystems.SideElevator;
import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.frc.subsystems.Lights;
import com.wcp.frc.subsystems.Scores;

import edu.wpi.first.wpilibj.XboxController;


public class Controls {
    SuperStructure s;

    double speed;
    
    

    XboxController Driver;
    XboxController CoDriver;
    double yaw;
    Swerve swerve;
    double Acelerator = .6;
    double autorotate = .2;
    double rotate = 0;
    double ignore = 1;

    double toggle = 0;
    Vision vision;
    boolean pick = false;
    boolean pickup = false;
    boolean hooman;
    boolean hoomanactive;
    int pipeline;
    double elevatorstate;
    boolean cube= true;

    ButtonCheck driverLeftTrigger = new ButtonCheck(.5);
    ButtonCheck driverRightTrigger = new ButtonCheck(.5);
    ButtonCheck driverLeftBumper = new ButtonCheck();
    ButtonCheck driverRightBumper = new ButtonCheck();
    ButtonCheck driverLeftStick = new ButtonCheck();
    ButtonCheck driverAButton = new ButtonCheck();
    ButtonCheck driverXButton = new ButtonCheck();
    ButtonCheck driverYButton = new ButtonCheck();
    ButtonCheck driverBButton = new ButtonCheck();
    ButtonCheck driverRightStickDown = new ButtonCheck();
    ButtonCheck driverStartButton = new ButtonCheck();
    ButtonCheck driverDpadLeft = new ButtonCheck();
    ButtonCheck driverDpadUp = new ButtonCheck();
    ButtonCheck driverDpadRight = new ButtonCheck();
    ButtonCheck driverDpadDown = new ButtonCheck();


    ButtonCheck coDriverStart = new ButtonCheck();
    ButtonCheck coDriverAButton = new ButtonCheck();
    ButtonCheck coDriverBButton = new ButtonCheck();
    ButtonCheck coDriverYButton = new ButtonCheck();
    ButtonCheck coDriverXButton = new ButtonCheck();
    ButtonCheck codriverLeftBumper = new ButtonCheck();
    ButtonCheck codriverRightBumper = new ButtonCheck();
    ButtonCheck coDriverLeftTrigger = new ButtonCheck(.5);
    ButtonCheck coDriverRightTrigger = new ButtonCheck(.5);
    ButtonCheck coDriverLeftStickDown = new ButtonCheck();
    ButtonCheck coDriverRightStickDown = new ButtonCheck();
    ButtonCheck coDriverBackButton = new ButtonCheck();





    private static Controls instance = null;
    public static Controls getInstance() {
        if (instance == null)
            instance = new Controls();
        return instance;
    }

    public Controls() {
        Driver = new XboxController(Ports.XBOX_1);
        CoDriver = new XboxController(Ports.XBOX_2);
        swerve = Swerve.getInstance();

        

        
    }


    public void update() {
        vision = Vision.getInstance();
        s = SuperStructure.getInstance();




        double driverLeftXInput = -(((Driver.getLeftX())) * Acelerator);
        double driverLeftYInput = (Driver.getLeftY() * Acelerator);// drive
        double driverRightXInput = -((((Driver.getRightX() * ignore) + rotate) * 2) * Acelerator);// drive



        double coDriverLeftX = CoDriver.getLeftX();
        double coDriverLeftY = CoDriver.getLeftY();
        double coDriverRightY = CoDriver.getRightY();
        double coDriverRightX = CoDriver.getRightX();

         driverLeftTrigger.update(Driver.getLeftTriggerAxis());
         driverRightTrigger.update(Driver.getRightTriggerAxis());
         driverLeftBumper.update(Driver.getLeftBumper());
         driverRightBumper.update(Driver.getRightBumper());
         driverLeftStick.update(Driver.getLeftStickButton());
         driverAButton.update(Driver.getAButton());
         driverXButton.update(Driver.getXButton());
         driverYButton.update(Driver.getYButton());
         driverBButton.update(Driver.getBButton());
         driverRightStickDown.update(Driver.getRightStickButton());
         driverStartButton.update(Driver.getStartButton());
         driverDpadLeft.update(Driver.getPOV() == 270);
         driverDpadUp.update(Driver.getPOV() == 0);
         driverDpadRight.update(Driver.getPOV() == 90);
         driverDpadDown.update(Driver.getPOV() == 180);
    
    
         coDriverStart.update(CoDriver.getStartButton());
         coDriverAButton.update(CoDriver.getAButton());
         coDriverBButton.update(CoDriver.getBButton());
         coDriverYButton.update(CoDriver.getYButton());
         coDriverXButton.update(CoDriver.getXButton());
         codriverLeftBumper.update(CoDriver.getLeftBumper());
         codriverRightBumper.update(CoDriver.getRightBumper());
         coDriverLeftTrigger.update(CoDriver.getLeftTriggerAxis());
         coDriverRightTrigger.update(CoDriver.getRightTriggerAxis());
         coDriverLeftStickDown.update(CoDriver.getLeftStickButton());
         coDriverRightStickDown.update(CoDriver.getLeftStickButtonPressed());
         coDriverBackButton.update(CoDriver.getBackButton());

    
        if(driverAButton.isPressed()) 
           s.setHeight(SuperStructure.PreState.LOW);
        if(driverBButton.isPressed())
           s.setHeight(SuperStructure.PreState.HOOMAN);
        if(driverXButton.isPressed())
           s.setHeight(SuperStructure.PreState.MID);
        if(driverYButton.isPressed())
           s.setHeight(SuperStructure.PreState.HIGH);
        if(driverLeftStick.isPressed())
           s.setPiece();
        if(driverLeftTrigger.isPressed())
           s.scoreState();
        else if(driverLeftTrigger.isReleased() && s.elevatorIsLocked())
           s.scoreReleaseState();
        else if(!driverLeftBumper.isActive() && !driverRightBumper.isActive() && !driverLeftTrigger.isActive() && !driverRightTrigger.isActive() && !s.elevatorIsLocked()){
           s.clearQueues();
           Acelerator = 1;
        }
        if(driverRightTrigger.isPressed()){
            s.aimState(driverLeftBumper.isPressed(), driverRightBumper.isPressed());
        }
        else if(driverRightTrigger.isActive()){
            swerve.updateOffset(driverLeftBumper.isPressed(), driverRightBumper.isPressed());            
        }
        else if(driverLeftBumper.isPressed())
           s.intakeState(SuperStructure.PreState.HOOMAN);
        else if(driverStartButton.isPressed())
            swerve.resetOdometry(vision.getPose(),swerve.getRobotHeading());
        
        else if(driverLeftBumper.isPressed())
           s.intakeState(SuperStructure.PreState.CHUTE);
        else if(!driverLeftBumper.isActive() && !driverRightBumper.isActive() && !driverLeftTrigger.isActive() && !driverRightTrigger.isActive() && !s.elevatorIsLocked()){
           s.clearQueues();
           Acelerator = 1;
        }
        else
           Acelerator = .5;
        


       s.requestSwerveInput(new Translation2d(driverLeftYInput, driverLeftXInput), driverRightXInput);



       
       
       

       swerve.sendInput(driverLeftYInput, driverLeftXInput, driverRightXInput);

      

        
}

public class ButtonCheck{
        double threshold;
        Boolean[] input = new Boolean[2];
            
        public ButtonCheck(double threshold){
            this.threshold = threshold;
            for(int i = 0; i < 1; i++){
                input[i] = false;
            }
        } 
        public ButtonCheck(){
            this.threshold = 0;
            for(int i = 0; i < 1; i++){
                input[i] = false;
            }
        } 

       public void update(double input){
            this.input[1] = this.input[0];
        if(input>threshold)
            this.input[0] = true;
        else
            this.input[0] = false;

       }

       public void update(boolean input){
            this.input[1] = this.input[0];
            if(input)
                this.input[0] = true;
            else
                this.input[0] = false;
       }

       public boolean isPressed(){
        return input[0] && !input[1];
       }

       public boolean isReleased(){
        return !input[0] && input[1];
       }

       public boolean isActive(){
         return input[0];
       }
       public void setThreshhold(double threshold){
         this.threshold = threshold;
       }
       }
}
