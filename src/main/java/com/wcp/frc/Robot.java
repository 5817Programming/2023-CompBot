// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Arrays;

import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import com.wcp.frc.subsystems.SubsystemManager;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.Lights;
import com.wcp.frc.Autos.Auto6;
import com.wcp.frc.subsystems.Arm;
import com.wcp.frc.subsystems.Balancing;
import com.wcp.frc.subsystems.SideElevator;
import com.wcp.frc.subsystems.Elevator;
import com.wcp.frc.subsystems.Intake;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
//https://github.com/Mechanical-Advantage/AdvantageKit/releases/latest/download/AdvantageKit.json

//https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {

  // private RobotContainer robotContainer = new RobotContainer();

  // RobotContainer robotContainer = new RobotContainer();
  Controls controls;
  SubsystemManager subsystemManager;
  Swerve swerve;
  double yaw;
  Balancing balancing = new Balancing();
  Elevator elevator = new Elevator();
  Vision vision;
  Arm arm = new Arm();
  Lights lights;
  Gyro pigeon;

  @Override
  public void robotInit() {
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

      // Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    

    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic
    // Timestamps" in the "Understanding Data Flow" page
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may
                                  // be added.
    swerve = Swerve.getInstance();
    swerve.zeroModules();


    controls = Controls.getInstance();
    swerve = Swerve.getInstance();
    vision = Vision.getInstance();
    
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
        Swerve.getInstance(), Arm.getInstance(), Elevator.getInstance(), Intake.getInstance(), Lights.getInstance(), SideElevator.getInstance(), Vision.getInstance()));
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
    balancing.toggle = false;
    swerve = Swerve.getInstance();
    swerve.fieldzeroSwerve();
    swerve.sendInput(0, 0,0);
    swerve.stop();
    // // schedule the autonomous command (example)
    // if (autoChooser.getSelected() != null) {
    // autoChooser.getSelected().schedule();
    // }
    new Auto6(swerve).schedule();

    startime = Timer.getFPGATimestamp();
      
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // switch (m_autoSelected) {
    // case kCustomAuto:
    // // Put custom auto code here
    // break;
    // case kDefaultAuto:
    // default:
    // // Put default auto code here
    // // break;
    swerve.updateTrajectory();

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // scores.zero();
    balancing.toggle = false;
    swerve = Swerve.getInstance();
    swerve.fieldzeroSwerve();
    swerve.sendInput(0, 0,0);

    // vision.range(1);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    controls.update();
    //swerve.update(Timer.getFPGATimestamp());
  }

  /** This function is called once when the robot is disabled. */

  @Override
  public void disabledInit() {
    subsystemManager.stopSubsystems();
    arm.setPercentOutput(0);
    elevator.my_PercentOutput(0);
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
