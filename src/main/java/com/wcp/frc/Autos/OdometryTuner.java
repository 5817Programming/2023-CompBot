package com.wcp.frc.Autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class OdometryTuner extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();

    PathPlannerTrajectory meter = PathPlanner.loadPath("1meter", new PathConstraints(4, 4));
    PathPlannerTrajectory ymeter = PathPlanner.loadPath("YTUNER", new PathConstraints(4, 4));

    @Override
    public void auto() {
        s.trajectoryState(ymeter,false);
    }
}