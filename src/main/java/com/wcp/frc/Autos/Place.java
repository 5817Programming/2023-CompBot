package com.wcp.frc.Autos;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SuperStructure;


public class Place extends AutoBase{
    SuperStructure s = SuperStructure.getInstance();

 

    @Override
    public void auto() {
        s.scoreState(SuperStructure.PreState.HIGH, true);
    }
}