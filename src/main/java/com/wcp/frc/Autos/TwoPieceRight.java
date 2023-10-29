package com.wcp.frc.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class TwoPieceRight extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();

    PathPlannerTrajectory maderaPT1 = PathPlanner.loadPath("2PieceBumpSide", new PathConstraints(4, 3));
    PathPlannerTrajectory maderaPT2 = PathPlanner.loadPath("2PieceBumpSide PT2", new PathConstraints(4, 3));


    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(maderaPT1,false);
        s.intakeState(SuperStructure.PreState.GROUND, true,true);
        s.trajectoryState(maderaPT2,false);
        s.scoreState(SuperStructure.PreState.MID, true);
        // s.balanceState();
    }

    
    


}