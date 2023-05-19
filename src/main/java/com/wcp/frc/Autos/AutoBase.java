// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.Autos;

import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve;

/** Add your docs here. */
public abstract class AutoBase {

    public abstract void auto();
    public void runAuto(){
        auto();

    }
    public void stopAuto(){
        SuperStructure.getInstance().clearQueues();
        Swerve.getInstance().setState(Swerve.State.MANUAL);
    }
}
