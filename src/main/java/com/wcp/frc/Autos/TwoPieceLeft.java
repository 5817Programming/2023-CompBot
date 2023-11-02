package com.wcp.frc.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class TwoPieceLeft extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();

    PathPlannerTrajectory maderaPT1 = PathPlanner.loadPath("OnePieceRight", new PathConstraints(4, 3));
    PathPlannerTrajectory maderaPT2 = PathPlanner.loadPath("OnePieceRight PT2", new PathConstraints(4, 3));
    PathPlannerTrajectory maderaPT3 = PathPlanner.loadPath("OnePieceRight PT3", new PathConstraints(4, 6));


    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(maderaPT1,false);
        s.intakeState(SuperStructure.PreState.GROUND, true,true);
        s.trajectoryState(maderaPT2,false);
        s.outtakeState(true);
        s.trajectoryState(maderaPT3,false);
        // s.balanceState();
    }

    
    


}