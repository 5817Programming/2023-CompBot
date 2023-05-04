
package com.wcp.frc.Autos;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.Balancing;
import com.wcp.frc.subsystems.Intake;
import com.wcp.frc.subsystems.Scores;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.lib.geometry.Rotation2dd;
import com.wcp.lib.geometry.Translation2dd;


public class Auto6 extends SequentialCommandGroup {



  
  public Auto6(Swerve swerve) {

    Balancing balence = new Balancing();
    Intake intake = new Intake();
    List<Translation2d> eventTimings = 
    Arrays.asList(
    new Translation2d(1.9,4.52),
    new Translation2d(6.42,4.57),
    new Translation2d(1.9,4.52) ,
    new Translation2d(6.43,3.32)
    );
    
    List<Double> waitTimings = Arrays.asList(3.0,3.0,3.0);
    List<Command> events = Arrays.asList(
    new InstantCommand(() -> intake.setPercentOutput(1)),
    new InstantCommand(() -> intake.setPercentOutput(0.5)),
    new InstantCommand(() -> intake.setPercentOutput(0)));

    


    PathPlannerTrajectory trajectory1 = (PathPlanner.loadPath("1meter", 4,4));//loads "2peice path"
    trajectory1.getMarkers();
    Logger.getInstance().recordOutput("traj1", trajectory1);

    //, new InstantCommand(() -> balence.toggle = true), new InstantCommand(() -> balence.balance())
    Logger.getInstance().recordOutput("trajstart", trajectory1.getInitialHolonomicPose());
    SequentialCommandGroup auto = new SequentialCommandGroup(
    new InstantCommand(() -> swerve.setTrajectory(trajectory1,eventTimings,events,waitTimings)),
    new InstantCommand(()->swerve.startPath(.8,true)),
    new WaitUntilCommand(swerve.TrajectoryisFinished()),
    new InstantCommand(() -> balence.toggle = true), new InstantCommand(() -> balence.balance()));
    addCommands(auto);
    
  }

}
