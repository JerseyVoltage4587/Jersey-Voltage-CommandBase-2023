/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.DriveConstants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.util.AsyncStructuredLogger;
import frc.robot.util.Gyro;

public class DriveBase extends SubsystemBase {
  private boolean m_isActive = true;
  public DifferentialDrive m_drive = null;
  static DriveBase m_Instance = null;
  private WPI_TalonSRX driveLeftTalon;
  private WPI_TalonSRX driveRightTalon;
  private VictorSPX driveLeftVictor;
  private VictorSPX driveRightVictor;
  private double LeftMotorLevel;
  private double RightMotorLevel;
  private double partialLeftInches = 0;
  private double partialRightInches = 0;
  private DriveBaseLoggingData m_loggingData;
  private AsyncStructuredLogger<DriveBaseLoggingData> m_logger;
  private long m_lastLogTime = 0;
  private String layout = "";
  private double leftTalonOldAmps   = 0;
  private double leftVictorOldAmps  = 0;
  private double rightTalonOldAmps  = 0;
  private double rightVictorOldAmps = 0;


  /**
   * Creates a new DriveBase.
   */
  public DriveBase() {
    if (m_isActive == false) {
      return;
    }

    driveLeftTalon = new WPI_TalonSRX(Constants.driveLeftTalon);
    driveRightTalon = new WPI_TalonSRX(Constants.driveRightTalon);
    driveLeftVictor = new VictorSPX(Constants.driveLeftVictor);
    driveRightVictor = new VictorSPX(Constants.driveRightVictor);
    // driveLeftTalon.configFactoryDefault();
    // driveRightTalon.configFactoryDefault();
    // driveLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // driveRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // driveLeftTalon.setSensorPhase(true);
    // driveRightTalon.setSensorPhase(true);
    // driveLeftTalon.config_kP(0, 0.001);
    // driveRightTalon.config_kP(0, 0.001);    
    // driveLeftTalon.config_kF(0, 1 / 3900.0);
    // driveRightTalon.config_kF(0, 1 / 3800.0);    
    // driveLeftTalon.setSelectedSensorPosition(0);
    // driveRightTalon.setSelectedSensorPosition(0);
    // driveLeftTalon.set(0);
    // driveRightTalon.set(0);
    // driveLeftTalon.setNeutralMode(NeutralMode.Brake);
    // driveRightTalon.setNeutralMode(NeutralMode.Brake);
    // driveLeftTalon.setInverted(false);
    // driveLeftVictor.setInverted(false);
    // driveRightTalon.setInverted(false);
    // driveRightVictor.setInverted(false);
    // driveLeftTalon.enableCurrentLimit(true);
    // driveLeftTalon.configContinuousCurrentLimit(30);
    driveLeftVictor.follow(driveLeftTalon);
    // driveRightTalon.enableCurrentLimit(true);
    // driveRightTalon.configContinuousCurrentLimit(30);
    driveRightVictor.follow(driveRightTalon);
    // driveLeftTalon.configOpenloopRamp(0.1);
    // driveRightTalon.configOpenloopRamp(0.1);
    m_drive = new DifferentialDrive(driveLeftTalon, driveRightTalon);
    m_drive.setSafetyEnabled(false);
    m_loggingData = new DriveBaseLoggingData();
    m_logger = new AsyncStructuredLogger<DriveBaseLoggingData>("DriveBase", /*forceUnique=*/false, DriveBaseLoggingData.class);
  }

  public static DriveBase getInstance() {
    if (m_Instance == null) {
      synchronized (DriveBase.class) {
        if (m_Instance == null) {
          m_Instance = new DriveBase();
        }
      }
    }
    return m_Instance;
  }

  public void teleopInit(){
		/* Disable all motor controllers */
		driveRightTalon.set(ControlMode.PercentOutput, 0);
		driveLeftTalon.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		driveRightTalon.configFactoryDefault();
		driveLeftTalon.configFactoryDefault();
		
		/* Set Neutral Mode */
		driveLeftTalon.setNeutralMode(NeutralMode.Brake);
		driveRightTalon.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor to a Quad Encoder*/
		driveLeftTalon.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
													DriveConstants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													DriveConstants.kTimeoutMs);					// Configuration Timeout

    driveRightTalon.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
													DriveConstants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													DriveConstants.kTimeoutMs);					// Configuration Timeout
		
		/* Configure output and sensor direction */
		driveLeftTalon.setInverted(false);
		driveLeftVictor.setInverted(false);
		driveLeftTalon.setSensorPhase(true);
		driveRightTalon.setInverted(true);
		driveRightVictor.setInverted(true);
		driveRightTalon.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		// driveRightTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, DriveConstants.kTimeoutMs);
		// driveRightTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, DriveConstants.kTimeoutMs);
		// driveRightTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, DriveConstants.kTimeoutMs);		
		// driveLeftTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, DriveConstants.kTimeoutMs);

		/* Configure neutral deadband */
		driveRightTalon.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);
		driveLeftTalon.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		 driveLeftTalon.configNominalOutputForward(0, DriveConstants.kTimeoutMs);
		 driveLeftTalon.configNominalOutputReverse(0, DriveConstants.kTimeoutMs);
		driveRightTalon.configNominalOutputForward(0, DriveConstants.kTimeoutMs);
		driveRightTalon.configNominalOutputReverse(0, DriveConstants.kTimeoutMs);
		driveLeftTalon.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		driveLeftTalon.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);
		driveRightTalon.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		driveRightTalon.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

		/* FPID Gains for velocity servo */
		driveRightTalon.config_kP(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kP, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kI(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kI, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kD(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kD, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kF(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kF, DriveConstants.kTimeoutMs);
		driveRightTalon.config_IntegralZone(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kIzone, DriveConstants.kTimeoutMs);
		driveRightTalon.configClosedLoopPeakOutput(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kPeakOutput, DriveConstants.kTimeoutMs);
		driveRightTalon.configAllowableClosedloopError(DriveConstants.kSlot_Velocit, 0, DriveConstants.kTimeoutMs);

		/* FPID Gains for turn servo */
		driveLeftTalon.config_kP(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kP, DriveConstants.kTimeoutMs);
		driveLeftTalon.config_kI(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kI, DriveConstants.kTimeoutMs);
		driveLeftTalon.config_kD(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kD, DriveConstants.kTimeoutMs);
		driveLeftTalon.config_kF(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kF, DriveConstants.kTimeoutMs);
		driveLeftTalon.config_IntegralZone(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kIzone, DriveConstants.kTimeoutMs);
		driveLeftTalon.configClosedLoopPeakOutput(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kPeakOutput, DriveConstants.kTimeoutMs);
		driveLeftTalon.configAllowableClosedloopError(DriveConstants.kSlot_Velocit, 0, DriveConstants.kTimeoutMs);
		
    driveLeftTalon.enableCurrentLimit(true);
    driveLeftTalon.configContinuousCurrentLimit(30, DriveConstants.kTimeoutMs);
    driveLeftTalon.configPeakCurrentLimit(40, DriveConstants.kTimeoutMs);
    driveLeftTalon.configPeakCurrentDuration(150, DriveConstants.kTimeoutMs);
    driveRightTalon.enableCurrentLimit(true);
    driveRightTalon.configContinuousCurrentLimit(30, DriveConstants.kTimeoutMs);
    driveRightTalon.configPeakCurrentLimit(40, DriveConstants.kTimeoutMs);
    driveRightTalon.configPeakCurrentDuration(150, DriveConstants.kTimeoutMs);

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		driveRightTalon.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
		driveLeftTalon.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);

    driveRightTalon.selectProfileSlot(DriveConstants.kSlot_Velocit, DriveConstants.PID_PRIMARY);
    driveLeftTalon.selectProfileSlot(DriveConstants.kSlot_Velocit, DriveConstants.PID_PRIMARY);

		/* Initialize */
		zeroDriveSensors(true);
	}



  public void autoInit(){
		/* Disable all motor controllers */
    if (true)
      return;

		driveRightTalon.set(ControlMode.PercentOutput, 0);
		driveLeftTalon.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		driveRightTalon.configFactoryDefault();
		driveLeftTalon.configFactoryDefault();
		
		/* Set Neutral Mode */
		driveLeftTalon.setNeutralMode(NeutralMode.Brake);
		driveRightTalon.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor to a Quad Encoder*/
		driveLeftTalon.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
													DriveConstants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													DriveConstants.kTimeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		driveRightTalon.configRemoteFeedbackFilter(driveLeftTalon.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												DriveConstants.REMOTE_1,						// Source number [0, 1]
												DriveConstants.kTimeoutMs);						// Configuration Timeout
		
		/* Setup Sum signal to be used for Distance */
		driveRightTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, DriveConstants.kTimeoutMs);	// Feedback Device of Remote Talon
		driveRightTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, DriveConstants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		driveRightTalon.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, DriveConstants.kTimeoutMs);
		driveRightTalon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, DriveConstants.kTimeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		driveRightTalon.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													DriveConstants.PID_PRIMARY,
													DriveConstants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		driveRightTalon.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														DriveConstants.PID_PRIMARY,		// PID Slot of Source 
														DriveConstants.kTimeoutMs);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		driveRightTalon.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													DriveConstants.PID_TURN, 
													DriveConstants.kTimeoutMs);
		
		/* Don't scale the Feedback Sensor (use 1 for 1:1 ratio) */
		driveRightTalon.configSelectedFeedbackCoefficient(	1, DriveConstants.PID_TURN, DriveConstants.kTimeoutMs);
		
		/* Configure output and sensor direction */
		driveLeftTalon.setInverted(false);
		driveLeftVictor.setInverted(false);
		driveLeftTalon.setSensorPhase(true);
		driveRightTalon.setInverted(true);
		driveRightVictor.setInverted(true);
		driveRightTalon.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		driveRightTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, DriveConstants.kTimeoutMs);
		driveRightTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, DriveConstants.kTimeoutMs);
		driveRightTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, DriveConstants.kTimeoutMs);		
		driveLeftTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, DriveConstants.kTimeoutMs);

		/* Configure neutral deadband */
		driveRightTalon.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);
		driveLeftTalon.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		driveLeftTalon.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		driveLeftTalon.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);
		driveRightTalon.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		driveRightTalon.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

		/* FPID Gains for velocity servo */
		driveRightTalon.config_kP(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kP, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kI(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kI, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kD(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kD, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kF(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kF, DriveConstants.kTimeoutMs);
		driveRightTalon.config_IntegralZone(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kIzone, DriveConstants.kTimeoutMs);
		driveRightTalon.configClosedLoopPeakOutput(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kPeakOutput, DriveConstants.kTimeoutMs);
		driveRightTalon.configAllowableClosedloopError(DriveConstants.kSlot_Velocit, 0, DriveConstants.kTimeoutMs);

		/* FPID Gains for turn servo */
		driveRightTalon.config_kP(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kP, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kI(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kI, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kD(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kD, DriveConstants.kTimeoutMs);
		driveRightTalon.config_kF(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kF, DriveConstants.kTimeoutMs);
		driveRightTalon.config_IntegralZone(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kIzone, DriveConstants.kTimeoutMs);
		driveRightTalon.configClosedLoopPeakOutput(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kPeakOutput, DriveConstants.kTimeoutMs);
		driveRightTalon.configAllowableClosedloopError(DriveConstants.kSlot_Turning, 0, DriveConstants.kTimeoutMs);
		
    driveLeftTalon.enableCurrentLimit(true);
    driveLeftTalon.configContinuousCurrentLimit(30, DriveConstants.kTimeoutMs);
    driveLeftTalon.configPeakCurrentLimit(40, DriveConstants.kTimeoutMs);
    driveLeftTalon.configPeakCurrentDuration(150, DriveConstants.kTimeoutMs);
    driveRightTalon.enableCurrentLimit(true);
    driveRightTalon.configContinuousCurrentLimit(30, DriveConstants.kTimeoutMs);
    driveRightTalon.configPeakCurrentLimit(40, DriveConstants.kTimeoutMs);
    driveRightTalon.configPeakCurrentDuration(150, DriveConstants.kTimeoutMs);

    
		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		driveRightTalon.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
		driveRightTalon.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		driveRightTalon.configAuxPIDPolarity(false, DriveConstants.kTimeoutMs);

		/* Initialize */

		zeroDriveSensors(true);
	}


  //Left Motor Variables
    public void setLeftVolts(double v) {
      driveLeftTalon.setVoltage(v);
    }

    public double getLeftMotorLevel() {
      if (m_isActive == false) {
        return -1;
      }
      return LeftMotorLevel;
    }

    public void setLeftMotorLevel(double x) {
      if (m_isActive == false) {
        return;
      }
      LeftMotorLevel = x;
      driveLeftTalon.set(LeftMotorLevel);
    }

    public double getLeftEncoder() {
      if (m_isActive == false) {
        return 0;
      }
      return driveLeftTalon.getSelectedSensorPosition() / 4096;
    }

    public double getLeftDistanceInches() {
      return getLeftEncoder() * Constants.DriveBaseWheelDiameter * Math.PI;
    }

    public double getPartialLeftInches() {
      return partialLeftInches;
    }

  //Right Motor Variables
    public void setRightVolts(double v) {
      driveRightTalon.setVoltage(v);
    }

    public double getRightMotorLevel() {
      if (m_isActive == false) {
        return -1;
      }
      return RightMotorLevel;
    }

    public void setRightMotorLevel(double x) {
      if (m_isActive == false) {
        return;
      }
      RightMotorLevel = x;
      driveRightTalon.set(RightMotorLevel);
    }

    public double getRightEncoder() {
      if (m_isActive == false) {
        return 0;
      }
      return driveRightTalon.getSelectedSensorPosition() / 4096;
    }

    public double getRightDistanceInches() {
      return getRightEncoder() * Constants.DriveBaseWheelDiameter * Math.PI;
    }

    public double getAverageDistanceInches() {
      return (Math.abs(getLeftDistanceInches()) + Math.abs(getRightDistanceInches()) / 2);
    }

    public double getPartialRightInches() {
      return partialRightInches;
    }

  public void setLayout(String l) {
    layout = l;
  }

  public String getLayout() {
    return layout;
  }

  public void setSafetyEnabled(boolean x){
    if (m_isActive == false) {
      return;
    }
    m_drive.setSafetyEnabled(x);
  }

  public void zeroDriveSensors(boolean gyro) {
    if (m_isActive == false) {
      return;
    }
		driveLeftTalon.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
		driveRightTalon.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
    if (gyro) {
      Gyro.getInstance();
      Gyro.reset();
    }
  }

  public void arcadeDrive(double forward, double turn) {
    if (m_isActive == false) {
      return;
    }
    m_drive.arcadeDrive(forward, turn);
  }

  private double getRateOfChange(double initialValue, double finalValue, long initialTime, long finalTime) {
    return (finalValue - initialValue) / (finalTime - initialTime);
  }

  public void MoveDistance(double x, double speed) {
    
    double averageInches = getAverageDistanceInches();
    if (Math.abs(averageInches) > Math.abs(x)) 
    {
      driveMotors(0, 0, true);
    } else {
      driveMotors(speed, 0, true);
    }
  }

  private ControlMode mode = ControlMode.Velocity;

  public void toggleMode(boolean velMode)
  {
    mode = !velMode ? ControlMode.Velocity : ControlMode.PercentOutput;
  }

      SlewRateLimiter throttleSlew = new SlewRateLimiter(2);
      SlewRateLimiter throttleSlewAuto = new SlewRateLimiter(1);
      SlewRateLimiter turnSlew = new SlewRateLimiter(4);

  public void driveMotors(double forward, double turn, boolean auton) {
    boolean slowTurn = OI.getInstance().j.getRawButton(8);
    boolean slowDrive = OI.getInstance().j.getRawButton(5);
    double scl = (mode == ControlMode.Velocity) ? 3200 : 1;
    double forwardLim = auton ? throttleSlewAuto.calculate(forward) : throttleSlew.calculate(forward);
    if(slowDrive) {
      forwardLim = -0.2;
    };
    double turnLim = turnSlew.calculate( slowTurn && ! auton ? turn / 2 : turn);
    double left = (forwardLim - turnLim);
    double right = (forwardLim + turnLim);
    left = Math.max(-1, Math.min(left, 1)) * scl;
    right = Math.max(-1, Math.min(right, 1)) * scl;

    driveLeftTalon.set(mode, -left);
    driveLeftVictor.follow(driveLeftTalon);
    driveRightTalon.set(mode, -right);
    driveRightVictor.follow(driveRightTalon);

    SmartDashboard.putNumber("Forward Level", forward);
    SmartDashboard.putNumber("Left Motor Level", getLeftMotorLevel());
  }

  @Override
  public void periodic() {
    if (m_isActive == false) {
      return;
    }



    // This method will be called once per scheduler run
    long now = System.nanoTime();
    double lastLeftPosition = m_loggingData.LeftPosition;
    double lastLeftVelocity = m_loggingData.LeftVelocity;
    double lastRightPosition = m_loggingData.RightPosition;
    double lastRightVelocity = m_loggingData.RightVelocity;

    m_loggingData.LeftMotorLevel = driveLeftTalon.get();
    m_loggingData.LeftMotor1_SupplyCurrent = driveLeftTalon.getOutputCurrent();
    m_loggingData.LeftMotor2_SupplyCurrent = Robot.getPDP().getCurrent(Constants.LeftSpark11PDP_Port);
    m_loggingData.LeftEncoderReading = getLeftEncoder();
    m_loggingData.LeftPosition = getLeftDistanceInches();
    m_loggingData.LeftVelocity = getRateOfChange(lastLeftPosition, m_loggingData.LeftPosition, m_lastLogTime, now);
    m_loggingData.LeftAcceleration = getRateOfChange(lastLeftVelocity, m_loggingData.LeftVelocity, m_lastLogTime, now);

    m_loggingData.RightMotorLevel = driveRightTalon.get();
    m_loggingData.RightMotor1_SupplyCurrent = driveRightTalon.getOutputCurrent();
    m_loggingData.RightMotor2_SupplyCurrent = Robot.getPDP().getCurrent(Constants.RightSpark21PDP_Port);
    m_loggingData.RightEncoderReading = getRightEncoder();
    m_loggingData.RightPosition = getRightDistanceInches();
    m_loggingData.RightVelocity = getRateOfChange(lastRightPosition, m_loggingData.RightPosition, m_lastLogTime, now);
    m_loggingData.RightAcceleration = getRateOfChange(lastRightVelocity, m_loggingData.RightVelocity, m_lastLogTime, now);

    m_loggingData.Heading = Gyro.getYaw();
    m_logger.queueData(m_loggingData);
    m_lastLogTime = now;

    leftTalonOldAmps   = 0.9 * leftTalonOldAmps   + 0.1 * Robot.getPDP().getCurrent(Constants.driveRightTalonPDPPort);
    leftVictorOldAmps  = 0.9 * leftVictorOldAmps  + 0.1 * Robot.getPDP().getCurrent(Constants.driveRightVictorPDPPort);
    rightTalonOldAmps  = 0.9 * rightTalonOldAmps  + 0.1 * Robot.getPDP().getCurrent(Constants.driveLeftTalonPDPPort);
    rightVictorOldAmps = 0.9 * rightVictorOldAmps + 0.1 * Robot.getPDP().getCurrent(Constants.driveLeftVictorPDPPort);
;

    SmartDashboard.putNumber("Left Vel", m_loggingData.LeftEncoderReading);
    // SmartDashboard.putNumber("Right Motor Level", getRightMotorLevel());
    // SmartDashboard.putNumber("Left Motor Level", getLeftMotorLevel());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Left Distance", getLeftDistanceInches());
    SmartDashboard.putNumber("Right Distance", getRightDistanceInches());
    SmartDashboard.putNumber("Right Talon Current", leftTalonOldAmps  );
    SmartDashboard.putNumber("Right Victor Current", leftVictorOldAmps );
    SmartDashboard.putNumber("Left Talon Current", rightTalonOldAmps );
    SmartDashboard.putNumber("Left Victor Current", rightVictorOldAmps);
    SmartDashboard.putNumber("Average Distance", getAverageDistanceInches());


  }

  public static class DriveBaseLoggingData {
    public double LeftMotorLevel;
    public double LeftMotor1_SupplyCurrent;
    public double LeftMotor1_StatorCurrent;
    public double LeftMotor2_SupplyCurrent;
    public double LeftEncoderReading;
    public double LeftPosition;
    public double LeftVelocity;
    public double LeftAcceleration;
    public double RightMotorLevel;
    public double RightMotor1_SupplyCurrent;
    public double RightMotor1_StatorCurrent;
    public double RightMotor2_SupplyCurrent;
    public double RightEncoderReading;
    public double RightPosition;
    public double RightVelocity;
    public double RightAcceleration;
    public double Heading;
  }
}
