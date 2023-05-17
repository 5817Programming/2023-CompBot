
package com.wcp.frc.Autos;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.Balancing;
import com.wcp.frc.subsystems.Intake;
import com.wcp.frc.subsystems.Scores;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.lib.geometry.Translation2d;


public class Auto6 extends SequentialCommandGroup {



  
  public Auto6(Swerve swerve) {
    Scores scores = new Scores();


    Balancing balence = new Balancing();
    Intake intake = new Intake();
    List<Translation2d> eventTimings = 
    Arrays.asList(
       new Translation2d(6.42,4.57),
       new Translation2d(2,4.52),
       new Translation2d(3.96, 2.95)
    );
    
    List<Double> waitTimings = Arrays.asList(3.0,1.0,1.0);
    List<Command> events = Arrays.asList
    (
      new SequentialCommandGroup(
      new InstantCommand(() -> scores.setHeight(Constants.ElevatorConstants.PICKUP,false)),
      new WaitCommand(1.5),
      new InstantCommand(() -> intake.setPercentOutput(-0.5)),
      new WaitCommand(.1),
      new InstantCommand(() -> intake.setPercentOutput(0)),
      new WaitCommand(.5),
      new InstantCommand(() -> scores.setHeight(Constants.ElevatorConstants.ZERO,false)),
      new WaitCommand(1)),
      new SequentialCommandGroup(
      new InstantCommand(() -> scores.setHeight(Constants.ElevatorConstants.MID_CUBE,false)),
      new InstantCommand(() -> intake.setPercentOutput(1)),
      new WaitCommand(.4),
      new InstantCommand(() -> intake.setPercentOutput(0)),
      new WaitCommand(.5),
      new InstantCommand(() -> scores.setHeight(Constants.ElevatorConstants.ZERO,false)),
      new WaitCommand(1)),
      new SequentialCommandGroup(
        new InstantCommand(() -> balence.toggle = true),
         new InstantCommand(() -> balence.balance())
        )
    );

    


    PathPlannerTrajectory trajectory1 = (PathPlanner.loadPath("MADERA", 4,4));//loads "2peice path"
    trajectory1.getMarkers();
    Logger.getInstance().recordOutput("traj1", trajectory1);

    //, new InstantCommand(() -> balence.toggle = true), new InstantCommand(() -> balence.balance())
    Logger.getInstance().recordOutput("trajstart", trajectory1.getInitialHolonomicPose());
    SequentialCommandGroup auto = new SequentialCommandGroup(
      new InstantCommand(() -> swerve.setTrajectory(trajectory1,eventTimings,events,waitTimings)),
      new InstantCommand(() -> scores.setHeight(Constants.ElevatorConstants.HIGH_CONE,false)),
      new WaitCommand(1.5),
      new InstantCommand(() -> intake.setPercentOutput(.5)),
      new WaitCommand(.1),
      new InstantCommand(() -> intake.setPercentOutput(0)),
      new WaitCommand(.5),
      new InstantCommand(() -> scores.setHeight(Constants.ElevatorConstants.ZERO,false)),
      new WaitCommand(1),
    new InstantCommand(()->swerve.startPath(1,true))
);
    addCommands(auto);
    
  }

}
