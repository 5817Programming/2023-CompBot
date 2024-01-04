package com.wcp.frc.Autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class Place extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();
    PathPlannerTrajectory path = PathPlanner.loadPath("test4", new PathConstraints(1, 1));
    PathPlannerTrajectory path2 = PathPlanner.loadPath("test3", new PathConstraints(1, 1));


    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
        s.trajectoryState(path, false);
        s.objectTargetState();
        s.intakeState(SuperStructure.PreState.GROUND, true,true);
        s.trajectoryState(path2, false);
        s.nodeState();
        s.scoreState(SuperStructure.PreState.HIGH, true);

    }
}