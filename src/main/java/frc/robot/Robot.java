// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.beans.Encoder;
import java.lang.Math;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.simulation.PowerDistributionDataJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private static final String kAuto = "auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static PowerDistribution m_PDP;
  DriveBase m_drive;
  OI oi;
  private Command m_autonomousCommand;

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */

  
  Encoder leftTalonEncoder = new Encoder();
  Encoder rightTalonEncoder = new Encoder();

  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(56);
  WPI_TalonSRX armMotor = new WPI_TalonSRX(8);

  SlewRateLimiter filter = new SlewRateLimiter(0.5, -0.5, 0);

  
  public static PowerDistribution getPDP() {
    if (m_PDP == null) {
      m_PDP = new PowerDistribution();
    }
    return m_PDP;
  }

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  /**/// CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  /**/// CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 6.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("mobility", kAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("do nothing", kNothingAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    CameraServer.startAutomaticCapture();
    CommandScheduler.getInstance().cancelAll();

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    // driveLeftTalon.setCANTimeout(100);
    // driveLeftVictor.follow(driveLeftTalon);
    // driveRightTalon.setCANTimeout(100);
    // driveRightVictor.follow(driveRightTalon);
    // driveLeftTalon.enableCurrentLimit(true);
    // driveLeftTalon.configContinuousCurrentLimit(30);
    // driveLeftVictor.follow(driveLeftTalon);
    // driveRightTalon.enableCurrentLimit(true);
    // driveRightTalon.configContinuousCurrentLimit(30);
    // driveRightVictor.follow(driveRightTalon);
    // driveLeftTalon.setOpenLoopRampRate(0.1);
    // driveLeftVictor.setOpenLoopRampRate(0.1);
    // driveRightTalon.setOpenLoopRampRate(0.1);
    // driveRightVictor.setOpenLoopRampRate(0.1);
    // driveLeftTalon.setInverted(false);
    // driveLeftVictor.setInverted(false);
    // driveRightTalon.setInverted(false);
    // driveRightVictor.setInverted(false);
    //driveLeftTalon.
    
    
    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    /**///arm.setInverted(true);
    /**///arm.setIdleMode(IdleMode.kBrake);
    /**///arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    /**///intake.setInverted(false);
    /**///intake.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    /**///arm.set(percent);
    /**///SmartDashboard.putNumber("arm power (%)", percent);
    /**///SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    /**///SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    /**///intake.set(percent);
    /**///intake.setSmartCurrentLimit(amps);
    /**///SmartDashboard.putNumber("intake power (%)", percent);
    /**///SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    /**///SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    CommandScheduler.getInstance().run();
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {

    CommandScheduler.getInstance().schedule(m_autonomousCommand);
    //zeroDriveSensors();

    //armMotor.set(0.4);
    //intakeMotor.set(1);
    
        // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);

    // if (m_autoSelected == kConeAuto) {
    //   autonomousIntakePower = INTAKE_OUTPUT_POWER;
    // } else if (m_autoSelected == kCubeAuto) {
    //   autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    // }

    // autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {

    // if (m_autoSelected == kNothingAuto) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    //   return;
    // }

    // double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    // if (timeElapsed < ARM_EXTEND_TIME_S) {
    //   setArmMotor(ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
    //   setArmMotor(-ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    // } else {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();

    armMotor.setSelectedSensorPosition(0);
    // zeroDriveSensors();
    // driveLeftTalon.setNeutralMode(NeutralMode.Brake);
    // driveLeftVictor.follow(driveLeftTalon);
    // driveRightTalon.setNeutralMode(NeutralMode.Brake);
    // driveRightVictor.follow(driveRightTalon);

    lastGamePiece = NOTHING;
  }
double armEncoder;
boolean lastPressed;
  @Override
  public void teleopPeriodic() {

    DriveBase.getInstance().driveMotors(OI.getInstance().j.getRawAxis(1),OI.getInstance().j.getRawAxis(2));
    

    // double armPower;
    // if (j.getRawButton(7)) {
    //   // lower the arm
    //   armPower = -ARM_OUTPUT_POWER;
    // } else if (j.getRawButton(5)) {
    //   // raise the arm
    //   armPower = ARM_OUTPUT_POWER;
    // } else {
    //   // do nothing and let it sit where it is
    //   armPower = 0.0;
    // }
    // setArmMotor(armPower);
    // boolean aVar = k.getRawButton(5);
    // boolean oaVar = k.getRawButton(6);
    // boolean iVar = k.getRawButton(7);
    // boolean oiVar = k.getRawButton(8);

    // intakeMotor.setNeutralMode(NeutralMode.Brake);
    // armMotor.setNeutralMode(NeutralMode.Brake);

    // if (iVar) {
    //   intakeMotor.set(1);
    // } else if (oiVar) {
    //   intakeMotor.set(-1);
    // } else {
    //   intakeMotor.set(0);
    // }
    // if (aVar) {
    //   armMotor.set(0.4);
    //   armEncoder = armMotor.getSelectedSensorPosition();
    //   SmartDashboard.putNumber("distance", armEncoder);
    // } else if (oaVar) {
    //   armMotor.set(-0.4);
    //   armEncoder = armMotor.getSelectedSensorPosition();
    //   SmartDashboard.putNumber("distance", armEncoder);
    // } else {
    //   armMotor.set(0);
    // }
    // iVar = false;
    // oiVar = false;
    // aVar = false;
    // oaVar = false;
  
    // double intakePower;
    // int intakeAmps;
    // if (j.getRawButton(8)) {
    //   // cube in or cone out
    //   intakePower = INTAKE_OUTPUT_POWER;
    //   intakeAmps = INTAKE_CURRENT_LIMIT_A;
    //   lastGamePiece = CUBE;
    // } else if (j.getRawButton(6)) {
    //   // cone in or cube out
    //   intakePower = -INTAKE_OUTPUT_POWER;
    //   intakeAmps = INTAKE_CURRENT_LIMIT_A;
    //   lastGamePiece = CONE;
    // } else if (lastGamePiece == CUBE) {
    //   intakePower = INTAKE_HOLD_POWER;
    //   intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    // } else if (lastGamePiece == CONE) {
    //   intakePower = -INTAKE_HOLD_POWER;
    //   intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    // } else {
    //   intakePower = 0.0;
    //   intakeAmps = 0;
    // }
    // setIntakeMotor(intakePower, intakeAmps);
 
    // /*
    //  * Negative signs here because the values from the analog sticks are backwards
    //  * from what we want. Forward returns a negative when we want it positive.
    //  */
    // double x =  -j.getRawAxis(1);
    // double y =  j.getRawAxis(2);
    // x = (x>0?1:-1) * x * x;
    // y = (y>0?1:-1) * y * y;
    // //y *= 0.7;

    // //x = filter.calculate(x);

    // setDriveMotors(x,y);
  }
}
