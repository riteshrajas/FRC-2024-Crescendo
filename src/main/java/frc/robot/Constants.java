// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Swerve.SwerveCANIDs;
import friarLib2.utility.PIDParameters;

/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants { 
    
    
    /**
    *  For Constants that are not in a subsystem
    */
    
    public static final double JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_DEADBAND = 0.05;
    
    public static final int PCM_CAN_ID = 1;
    public static final int PIGEON_IMU_ID = 19;
    
    public static final PneumaticsModuleType PCM_TYPE= PneumaticsModuleType.REVPH;
    public static final int TIMEOUT_MS = 30;

    /**
    * Constants for the Drivetrain
    */
    public static class Drive {
        /********** CAN ID's **********/
        public static final SwerveCANIDs FRONT_LEFT_MODULE_IDS = new SwerveCANIDs(32, 36, 61);
        public static final SwerveCANIDs FRONT_RIGHT_MODULE_IDS = new SwerveCANIDs(33, 37, 62);
        public static final SwerveCANIDs BACK_LEFT_MODULE_IDS = new SwerveCANIDs(34, 39, 60);
        public static final SwerveCANIDs BACK_RIGHT_MODULE_IDS = new SwerveCANIDs(40, 41, 59);
        
        /********** Module Translations **********/
        public static final Translation2d FRONT_LEFT_MODULE_TRANSLATION = new Translation2d(0.34671, 0.23241);
        public static final Translation2d FRONT_RIGHT_MODULE_TRANSLATION = new Translation2d(0.34671, -0.23241);
        public static final Translation2d BACK_LEFT_MODULE_TRANSLATION = new Translation2d(-0.34671, 0.23241);
        public static final Translation2d BACK_RIGHT_MODULE_TRANSLATION = new Translation2d(-0.34671, -0.23241);
        
        /********** Autonomous Motion Envelope **********/
        public static final double MAX_AUTON_SPEED = 2; // Meters/second
        public static final double MAX_AUTON_ACCELERATION = 2.5; // Meters/second squared
        public static final double MAX_AUTON_ANGULAR_SPEED = 400; // Degrees/second
        public static final double MAX_AUTON_ANGULAR_ACCELERATION = 200; // Degrees/second squared
        
        /********** Holonomic Controller Gains **********/
        public static final PIDConstants HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS = new PIDConstants(4, .75,0.5);
        public static final PIDConstants HOLONOMIC_CONTROLLER_PID_ROTATIONAL_CONSTRAINTS = new PIDConstants(5, 0,0);
        public static final PIDController HOLONOMIC_CONTROLLER_PID_X = new PIDController(HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS.kP, HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS.kI,HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS.kD);
        public static final PIDController HOLONOMIC_CONTROLLER_PID_Y = new PIDController(HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS.kP, HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS.kI,HOLONOMIC_CONTROLLER_PID_XY_CONSTRAINTS.kD);
        public static final ProfiledPIDController HOLONOMIC_CONTROLLER_PID_THETA = new ProfiledPIDController(HOLONOMIC_CONTROLLER_PID_ROTATIONAL_CONSTRAINTS.kP, HOLONOMIC_CONTROLLER_PID_ROTATIONAL_CONSTRAINTS.kI, HOLONOMIC_CONTROLLER_PID_ROTATIONAL_CONSTRAINTS.kD, new TrapezoidProfile.Constraints(MAX_AUTON_ANGULAR_SPEED, MAX_AUTON_ANGULAR_ACCELERATION));
        
        /******** PID Gains ********/
        public static final PIDController VISION_AIM_PID = new PIDController(0.15, 0.1, 0);
        
        /********** Teleop Control Adjustment **********/
        public static final double MAX_TELEOP_SPEED = 6; // Meters/second
        public static final double MAX_TELEOP_ROTATIONAL_SPEED = Math.toRadians(700); // Radians/second
        public static final double MAX_TELEOP_ACCELERATION = 10; // Maters/second squared
        public static final double MAX_TELEOP_DECELERATION = 11;
    }

    public class Intake 
    {
        /********** CAN ID's **********/
        public static final int INTAKE_MOTOR_ID = 4;
        public static final int PIVOT_MOTOR_ID = 10;
 
        /********** Amp Scoring **********/
        public static final double INTAKE_MOTOR_AMP_POWER = 1;

        /********** Intake  **********/
        public static final double INTAKE_MOTOR_INTAKE_POWER = 1;

        /********** Trap Scoring **********/
        public static final double INTAKE_MOTOR_TRAP_POWER = 1;

        /********** pivot **********/
        public static final double PIVOT_MOTOR_POWER = 1;

        /******** Misc ********/
        public static final double TargetThreshold = 10;

        /******** Pivot PID ********/
        public static final PIDParameters PIVOT_MOTOR_PID = new PIDParameters(0, "Pivot Motor PID", 0.02, 0.0, 0.25, 0.0, 50); //all of the stuff in this are placeholders

    }

    public class Indexer
    {
        /********** CAN ID's **********/
        public static final int INDEXER_MOTOR_ID = 100;

        /******** Indexer PID ********/
        public static final PIDParameters INDEXER_MOTOR_PID = new PIDParameters(0, "Pivot Motor PID", 0.02, 0.0, 0.25, 0.0, 50); //all of the stuff in this are placeholders

        /********** Indexer Motor Power **********/
        public static final double Indexer_MOTOR_POWER = 1;
    }

    public class Arm
    {
        /********** CAN ID's **********/
        public static final int ARM_MOTOR_ID = 99;
    }

    public class Shooter
    {
        /********** CAN ID's **********/
        public static final int SHOOTER1_MOTOR_ID = 98;
        public static final int SHOOTER2_MOTOR_ID = 97;
    }
}



