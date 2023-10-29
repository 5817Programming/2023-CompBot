package com.wcp.frc.Autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class OnePieceRight extends AutoBase{

    double speed = 6;
    SuperStructure s = SuperStructure.getInstance();

    PathPlannerTrajectory rightPT1 = PathPlanner.loadPath("RIGHT PT1", new PathConstraints(speed, speed));

    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(rightPT1,false);



    }
}