/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //CAN AddresAses

    public static final int driveLeftTalon = 17;
    public static final int driveRightTalon = 4;
    public static final int driveLeftVictor = 3;
    public static final int driveRightVictor = 5;

    public static final int intakeMotor = 56;
    public static final int armMotor = 8;
    
    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0.0313;
    public static final double kCruiseVelocity = 12000;
    public static final double kCruiseAcceleration = 12000;



    
    //Motor Stall Currents
    
    //Encoder Tics
    public static final int DriveBaseEncoderTics = 4096; //final
    
    //Wheel Diameters
    public static final double DriveBaseWheelDiameter = 6; //Inches //final
    public static final double ShooterWheelDiameter = 0; //Inches //TBD
    
    //Rotations
    public static final double ClimberForwardRotations = 0; //TBD
    public static final double ClimberBackwardRotations = 0; //TBD

    //Tolerance
    public static final double TrapezoidProfileTolerance = 1; //final

    //PID
        //Shooter
        public static final double kMaxSpeed = 0;
        public static final double kMaxAcceleration = 0;
        public static final double kSecondsPerCycle = 0;
/*        public static final double ksVoltsLeft = 0;
        public static final double kvVoltsLeft = 0;
        public static final double kaVoltsLeft = 0;*/
        public static final double kpDriveVel = 0;
/*        public static final double ksVoltsRight = 0;
        public static final double kvVoltsRight = 0;
        public static final double kaVoltsRight = 0;*/
        // Unit of gain uncertain - likely meter, possibly rotations.
        public static final double ksVoltsShooter = 0.5;//0.89343;
        public static final double kvVoltsShooter = 0.00142;//0.13196;
        public static final double kaVoltsShooter = 0.0016662;
    public final static double ksVolts = 0; //TBD
    public final static double kvVolts = 0; //TBD
    public final static double kaVolts = 0; //TBD

    //Modes
    public final static int IntakeOFF_MODE = 0;
    public final static int IntakeIN_MODE = 1;
    public final static int IntakeOUT_MODE = 2;
    public final static int ShooterON_MODE = 0;
    public final static int ShooterOFF_MODE = 1;
    public final static int ShooterBACK_MODE = 2;
    public final static int FrontCamera = 0; //front camera
    public final static int BackCamera = 1; //back camera

    public final static double climberVoltageChange = 0.001;
}