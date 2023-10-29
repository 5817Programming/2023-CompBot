// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.lib.util.PathFollower;
import com.wcp.frc.subsystems.Lights;
import com.google.flatbuffers.FlexBuffers.Map;
import com.wcp.frc.Autos.AutoBase;
import com.wcp.frc.Autos.OdometryTuner;
import com.wcp.frc.Autos.OnePieceBalanceCommunityMid;
import com.wcp.frc.Autos.OnePieceBalanceRight;
import com.wcp.frc.Autos.OnePieceLeftBalance;
import com.wcp.frc.Autos.OnePieceRight;
import com.wcp.frc.Autos.Place;
import com.wcp.frc.Autos.TwoPieceLeft;
import com.wcp.frc.Autos.TwoPieceRight;
import com.wcp.frc.subsystems.Arm;
import com.wcp.frc.subsystems.SideElevator;
import com.wcp.frc.subsystems.Elevator;
import com.wcp.frc.subsystems.Intake;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
//https://github.com/Mechanical-Advantage/AdvantageKit/releases/latest/download/AdvantageKit.json
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {


  // private RobotContainer robotContainer = new RobotContainer();

  // RobotContainer robotContainer = new RobotContainer();
  Controls controls;
  SubsystemManager subsystemManager;
  Swerve swerve;
  double yaw;
  Elevator elevator = Elevator.getInstance();
  Vision vision;
  Arm arm = Arm.getInstance();
  Lights lights;
  Gyro pigeon;
  public SendableChooser<AutoBase> autoChooser = new SendableChooser<>();

HashMap<String,AutoBase> autos = new HashMap<String,AutoBase>();
  @Override
  public void robotInit() {
    autos.put("2PieceLeft", new TwoPieceLeft());
    autos.put("1PieceBalanceCommunityMid", new OnePieceBalanceCommunityMid());
    autos.put("1PieceRight", new OnePieceRight());
    autos.put("1PieceBalanceRight",new OnePieceBalanceRight());
    autos.put("1PieceBalanceLeft", new OnePieceLeftBalance());
    autos.put("2PieceRight", new TwoPieceRight());

    for(HashMap.Entry<String, AutoBase> entry : autos.entrySet()) {
      String N = entry.getKey();
      AutoBase A = entry.getValue();
      autoChooser.addOption(N, A);
    }

    SmartDashboard.putData("Autonomous routine", autoChooser);
    
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

    Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may

    swerve = Swerve.getInstance();
    swerve.zeroModules();
    controls = Controls.getInstance();
    swerve = Swerve.getInstance();
    vision = Vision.getInstance();
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
        Swerve.getInstance(), SuperStructure.getInstance(), Arm.getInstance(), Elevator.getInstance(), Intake.getInstance(), Lights.getInstance(), SideElevator.getInstance(), Vision.getInstance()));
    }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * 
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and 
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
swerve.updatePose(Timer.getFPGATimestamp());
    elevator.getEncoder();
    arm.getEncoder();

    subsystemManager.updateSubsystems();
    subsystemManager.readSystemsPeriodicInputs();
    subsystemManager.writeSubsystemsPeriodicOutputs();
    subsystemManager.outputSystemsTelemetry();
    CommandScheduler.getInstance().run();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */

  double startime;


  @Override
  public void autonomousInit() {
    swerve = Swerve.getInstance();
    
    swerve.fieldzeroSwerve();
    swerve.sendInput(0, 0,0);
    swerve.stop();

    startime = Timer.getFPGATimestamp();
    
    if(autoChooser.getSelected() != null){
      autoChooser.getSelected().runAuto();
    }else{
      new Place().runAuto();
    }

      
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SuperStructure.getInstance().idleState();
    swerve = Swerve.getInstance();
    swerve.fieldzeroSwerve();




  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    controls.update();
  }

  /** This function is called once when the robot is disabled. */

  @Override
  public void disabledInit() {
    subsystemManager.stopSubsystems();
    arm.setPercentOutput(0);
    elevator.percentOutput(0);
    SuperStructure.getInstance().clearQueues();
    PathFollower.getInstance().resetTimer();
 //   scores.setHeight(Constants.ElevatorConstants.ZERO,true);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
