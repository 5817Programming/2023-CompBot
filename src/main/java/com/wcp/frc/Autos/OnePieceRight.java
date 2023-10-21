package com.wcp.frc.Autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class OnePieceRight extends AutoBase{

    double speed = 6;
    SuperStructure s = SuperStructure.getInstance();

    PathPlannerTrajectory rightPT1 = PathPlanner.loadPath("OnePieceRight", new PathConstraints(speed, speed));
    PathPlannerTrajectory rightPT2 = PathPlanner.loadPath("OnePieceRight PT2", new PathConstraints(speed, speed));
    PathPlannerTrajectory rightPT3 = PathPlanner.loadPath("OnePieceRight PT3", new PathConstraints(speed, speed));

    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(rightPT1,true);
        s.waitForTrajectoryState(0.5);
        s.intakeState(SuperStructure.PreState.GROUND, true);
        s.waitForTrajectoryState(1);
        s.trajectoryState(rightPT2,false);
        s.outtakeState(true);
        s.trajectoryState(rightPT3,false);
        s.balanceState();


    }
}