package com.wcp.frc.Autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class OnePieceBalanceCommunityMid extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    double speed = 4;
    PathPlannerTrajectory midPT1 = PathPlanner.loadPath("MID PT1", new PathConstraints(speed, speed));
    PathPlannerTrajectory midPT2 = PathPlanner.loadPath("MID PT2", new PathConstraints(speed, speed));
    PathPlannerTrajectory midPT3 = PathPlanner.loadPath("MID PT3", new PathConstraints(speed, speed));


    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(midPT1,false);
        s.trajectoryState(midPT2,false);
        s.trajectoryState(midPT3,false);
        s.balanceState();
    }
}