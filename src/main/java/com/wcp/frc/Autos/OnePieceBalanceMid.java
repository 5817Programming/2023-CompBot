package com.wcp.frc.Autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class OnePieceBalanceMid extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();

    PathPlannerTrajectory midPT1 = PathPlanner.loadPath("MID PT1", new PathConstraints(4, 4));
    PathPlannerTrajectory midPT2 = PathPlanner.loadPath("MID PT2", new PathConstraints(4, 4));
    PathPlannerTrajectory midPT3 = PathPlanner.loadPath("MID PT3", new PathConstraints(4, 4));


    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(midPT1);
        s.trajectoryState(midPT2);
        s.trajectoryState(midPT3);
        s.balanceState();
    }
}
