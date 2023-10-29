package com.wcp.frc.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class OnePieceLeftBalance extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();

    PathPlannerTrajectory maderaPT1 = PathPlanner.loadPath("OnePieceRight", new PathConstraints(6, 4));
    PathPlannerTrajectory maderaPT2 = PathPlanner.loadPath("OnePieceBalanceLeft", new PathConstraints(6, 4));


    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(maderaPT1,false);
        s.intakeState(SuperStructure.PreState.GROUND, true,true);
        s.trajectoryState(maderaPT2,false);
        s.balanceState();
    }

    

}