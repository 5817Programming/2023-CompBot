// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Arrays;
import java.util.List;

import com.wcp.lib.geometry.Translation2dd;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final int kCANTimeoutMs = 20;//The refresh rate of the periodic looper


    public static final double kRobotBaseWidth = 29.0; //The Robot Wheel Base Width
    public static final double kRobotBaseLength = 29.0; 
    public static final double mRobotBaseWidth = Units.inchesToMeters(29); //The Robot Wheel Base Width
    public static final double mRobotBaseLength = Units.inchesToMeters(29);//The Robot Wheel Base Length

    public static final double kOuterWheelDriveDiameter = 4.0;

    //Sets the motor on a 2Dplane
    public static final Translation2d[] modulePositions = new Translation2d[] {
        new Translation2d(mRobotBaseWidth / 2, mRobotBaseLength / 2),
        new Translation2d(mRobotBaseWidth / 2, -mRobotBaseLength / 2),
        new Translation2d(-mRobotBaseWidth / 2, mRobotBaseLength / 2),
        new Translation2d(-mRobotBaseWidth / 2, -mRobotBaseLength / 2),
      
    };
    //The positions of the modules, relative to the robot's center
    public static final Translation2dd kFrontRightPosition = new Translation2dd(kRobotBaseWidth / 2, kRobotBaseLength / 2);
    public static final Translation2dd kFrontLeftPosition = new Translation2dd(kRobotBaseWidth / 2, -kRobotBaseLength / 2);
    public static final Translation2dd kRearLeftPosition = new Translation2dd(-kRobotBaseWidth / 2, -kRobotBaseLength / 2);
    public static final Translation2dd kRearRightPosition = new Translation2dd(-kRobotBaseWidth / 2, kRobotBaseLength / 2);
    public static final Translation2dd mFrontRightPosition = new Translation2dd(mRobotBaseWidth / 2, mRobotBaseLength / 2);
    public static final Translation2dd mFrontLeftPosition = new Translation2dd(mRobotBaseWidth / 2, -mRobotBaseLength / 2);
    public static final Translation2dd mRearLeftPosition = new Translation2dd(-mRobotBaseWidth / 2, -mRobotBaseLength / 2);
    public static final Translation2dd mRearRightPosition = new Translation2dd(-mRobotBaseWidth / 2, mRobotBaseLength / 2);
    
    public static final List<Translation2dd> kModulePositions = Arrays.asList(kFrontRightPosition, kFrontLeftPosition, kRearLeftPosition, kRearRightPosition); 


    public static double kSwerveRotationMaxSpeed = 12720 * 0.8;//trigger changes this in controls kinda ghetto ngl

    	//Scrub Factors
	public static final boolean kSimulateReversedCarpet = false;
	public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
	public static final double kXScrubFactor = 1.0 / (1.0 - (9549.0 / 293093.0));
	public static final double kYScrubFactor = 1.0 / (1.0 - (4.4736 / 119.9336));
    
    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final double kSwerveMaxspeedMPS = 10;
    public static final double SwerveMaxspeedMPS = 4.5;
    public static final double kSwerveDriveMaxSpeed = 22000.0; //The theoretical max speed(For the Falcon 500s)
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;

    public static final double kSwerveRotationReduction = Options.rotationRatio; //The Module to Motor Ratio(i.e, amount the rotation motor rotates, for every one rotation for the module)
    public static final double kSwerveWheelReduction = Options.driveRatio; //The Wheel to Motor Ratio(i.e, amount the drive motor rotates, for every one rotation for the wheel)
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    public static final double kSwerveDriveEncoderResolution = 2048.0; 

    public static final double kSwerveWheelDiameter = 4; //inches 

    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * 2048;
	public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    public static final double kWheelCircumference = Units.inchesToMeters(kSwerveWheelDiameter*Math.PI)*.9;


    ///The absolute starting postion for each module
    //originally +180 to each
    public static final double kFrontRightStartingEncoderPosition = -82.2; //-354.950352
    public static final double kFrontLeftStartingEncoderPosition = -290.9; //-263.094811
    public static final double kRearLeftStartingEncoderPosition = -39.84; //-121.094031
    public static final double kRearRightStartingEncoderPosition = -108.1; //-355.170825    
        
        
    public static final class ElevatorConstants {

        //IN TIK
        public static final double MAX_UP = 78000;
        public static final double HOOMAN_CONE = 59700;
        public static final double HOOMAN_CUBE = 57000;
        public static final double MAX_DOWN = 800;

        public static final double LOW_CONE = MAX_UP * .1 ;
        public static final double MID_CONE = 27000;
        public static final double HIGH_CONE = 67000;
        public static final double LOW_CUBE = MAX_UP * .1 ;
        public static final double MID_CUBE = 1100;
        public static final double HIGH_CUBE = 65000;


        public static final double PICKUP = MAX_DOWN+5;
        public static final double ZERO = 801;
        public static final double HOLD = MAX_DOWN;//800
       
    }

    
    public static final class SideElevatorConstants{

        // IN TICKS 
        
        public static final double MAX_UP = 44000;

        public static final double MAX_DOWN = 0;

        public static final double LOW_CONE = MAX_UP * .1;
        
        public static final double MID_CONE =  1000;
        
        public static final double HIGH_CONE = 40000;

        public static final double LOW_CUBE = MAX_DOWN;
        
        public static final double MID_CUBE =  0;
        
        public static final double HIGH_CUBE = 44000;

     

        
        public static final double HOLD = 320;
        
        public static final double ZERO = MAX_DOWN;

        public static final double HOOMAN = 10;

        public static final double PICKUP = 0;

        public final static int ROTATION_PER_FOOT = 6;

    }

    public static final class ArmConstants {

        // IN TICKS

        public static final double MAX_UP_TICKS = 0;
        
        public static final double MAX_DOWN_TICKS = -63605;
        
        public static final double MIDDLE_CONE = -26000;

        public static final double HIGH_CONE = -30000;

        public static final double HOOMAN = -29000  ;

        public static final double LOW_SCORE_CONE = MIDDLE_CONE;

        public static final double MIDDLE_CUBE = -19624;

        public static final double HIGH_CUBE = -21018;

        public static final double LOW_SCORE_CUBE = 0;

        public static final double PICKUP = -60605;
        
        public static final double HOLD = -5000;//MAX_UP_TICKS-2000

        public static final double ZERO = MAX_UP_TICKS;
        
    }

    public static final class VisionConstants {
        
        public final static int CUBE_PIPELINE = 3;
        public final static int CONE_PIPELNE = 2;
        public final static int LOW_RETRO_PIPLINE = 3;
        public final static int APRIL_PIPLINE = 0;

        public final static double APRIL_HEIGHT_INCHES = 18.467;
        public final static double RETRO_HEIGHT_INCHES = 26.063;
        public final static double PICKUP_HEIGHT_INCHES = 0;
        public final static double HOOMAN_HEIGHT_INCHES = 27;
        public final static List<Double> HEIGHTS = Arrays.asList(APRIL_HEIGHT_INCHES, RETRO_HEIGHT_INCHES, HOOMAN_HEIGHT_INCHES,
            PICKUP_HEIGHT_INCHES);

        public final static double APRIL_OFFSET = 0;
        public final static double RETRO_OFFSET = .3;
        public final static double PICKUP_OFFSET = 0;
        public final static double HOOMAN_OFFSET = .4;
        public final static List<Double> OFFSETS = Arrays.asList(APRIL_OFFSET, RETRO_OFFSET, HOOMAN_OFFSET,
            PICKUP_OFFSET);

        // how many degrees back is your limelight rotated from perfectly vertical?
        public final static double LIMELIGHT_MOUNT_ANGLE_DEGREES = 1.4;

        // distance from the center of the Limelight lens to the floor
        public final static double LIMELIGHT_LENS_HEIGHT_INCHES = 28.6;
        public static final double lowerThreshold = 0;
        public static final double upperThreshold = 0;
    }

    public static final class LightConstants {
        public final static double NORMAL_LIGHT = -0.99;//rainbow light

        public final static double CUBE_LIGHT = 0.57;//purple light

        public final static double CONE_LIGHT = 0.65;//yellow light
    }
    

    
        
    public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int TIMEOUT_MILLISECONDS = 30;


    




    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI ;

        public static final double kPXController = 10;
        public static final double kPYController = 10;
        public static final double kPThetaController = 1;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static double zero=19241;
    public static double pickup =83492;
    public static final List<Double> scoresY =  Arrays.asList(
        0.46,
        1.04,
        1.57,
        2.16,
        2.72,
        3.25,
        3.84,
        4.39,
        4.94
    );
}


