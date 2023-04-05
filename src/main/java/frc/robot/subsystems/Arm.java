// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;
import frc.robot.Constants;
import frc.robot.OI;

public class Arm extends SubsystemBase {
  public boolean isActive = true;
  static Arm Instance = null;
  public WPI_TalonSRX armMotor;
  private int mode = Constants.IntakeOFF_MODE;
  private boolean deployed = false;

  private TimeInterpolatableBuffer<Double> FFTable;

  /** Creates a new Arm. */
  
  public Arm() {
    if (isActive == false) {
      return;
    }
    armMotor = new WPI_TalonSRX(8);
    armMotor.configFactoryDefault();
    armMotor.configContinuousCurrentLimit(20);
    armMotor.enableCurrentLimit(true);
    armMotor.configPeakCurrentDuration(150);
    armMotor.configPeakCurrentLimit(60);
    
    FFTable = TimeInterpolatableBuffer.createDoubleBuffer(40000);
    // m_Encoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
  }

  public static Arm getInstance() {
    if(Instance == null) {
      synchronized (Arm.class) {
        if(Instance == null) {
          Instance = new Arm();
        }
      }
    }
    return Instance;
  }

  public void robotInit() {
		/* setup some followers */
    armMotor.set(ControlMode.PercentOutput, 0);

		/* Factory default hardware to prevent unexpected behavior */
		armMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, ArmConstants.kPIDLoopIdx,
				ArmConstants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		armMotor.configNeutralDeadband(0.001, ArmConstants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		armMotor.setSensorPhase(true);
		armMotor.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ArmConstants.kTimeoutMs);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, ArmConstants.kTimeoutMs);
		armMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, ArmConstants.kTimeoutMs);
		armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, ArmConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		armMotor.configNominalOutputForward(0, ArmConstants.kTimeoutMs);
		armMotor.configNominalOutputReverse(0, ArmConstants.kTimeoutMs);
		armMotor.configPeakOutputForward(1, ArmConstants.kTimeoutMs);
		armMotor.configPeakOutputReverse(-1, ArmConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		armMotor.selectProfileSlot(ArmConstants.kSlotIdx, ArmConstants.kPIDLoopIdx);
		armMotor.config_kF(ArmConstants.kSlotIdx, ArmConstants.kGains.kF, ArmConstants.kTimeoutMs);
		armMotor.config_kP(ArmConstants.kSlotIdx, ArmConstants.kGains.kP, ArmConstants.kTimeoutMs);
		armMotor.config_kI(ArmConstants.kSlotIdx, ArmConstants.kGains.kI, ArmConstants.kTimeoutMs);
		armMotor.config_kD(ArmConstants.kSlotIdx, ArmConstants.kGains.kD, ArmConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		armMotor.configMotionCruiseVelocity(6000, ArmConstants.kTimeoutMs);
		armMotor.configMotionAcceleration(6000, ArmConstants.kTimeoutMs);
    armMotor.configMotionSCurveStrength(ArmConstants.kSmooth);

    int closedLoopTimeMs = 2;
		armMotor.configClosedLoopPeriod(0, closedLoopTimeMs, ArmConstants.kTimeoutMs);

    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		/* Zero the sensor once on robot boot up */
		armMotor.setSelectedSensorPosition(0, ArmConstants.kPIDLoopIdx, ArmConstants.kTimeoutMs);
	}
  
  public void setArm(ControlMode control, double level) {
    armMotor.set(control, level);
  }

  boolean closedLoop = false;

  private void checkOLRamp(boolean newClosedLoop)
  {
    if (newClosedLoop != closedLoop)
    {
      if (newClosedLoop)
        armMotor.configOpenloopRamp(0);
      else
        armMotor.configOpenloopRamp(0);
    }
    closedLoop = newClosedLoop;
  }

  public boolean atReverseLimit()
  {
    return armMotor.isRevLimitSwitchClosed() > 0;
  }

  public void armInside() {
    checkOLRamp(false);
  armMotor.set(ControlMode.PercentOutput, -0.6);
  }

  public void armOutside() {
    checkOLRamp(false);
    armMotor.set(ControlMode.PercentOutput, 0.6);
  }

  public void midCone() {
    checkOLRamp(true);
    armMotor.set(ControlMode.MotionMagic, 26000);
  }

  public void midCube() {
    checkOLRamp(true);
    armMotor.set(ControlMode.MotionMagic, 25000);
  }

  public void highCone() {
    checkOLRamp(true);
    armMotor.set(ControlMode.MotionMagic, 26000);
  }

  public void highCube() {
    checkOLRamp(true);
    armMotor.set(ControlMode.MotionMagic, 25000);
  }

  public void gotoPosition(double position, boolean adjust) {
    checkOLRamp(true);
    double adj = adjust ? OI.getInstance().k.getRawAxis(1) : 0;
    adj *= ArmConstants.adjScale;

    armMotor.set(ControlMode.MotionMagic, position - adj);
    SmartDashboard.putNumber("Arm Error", armMotor.getClosedLoopError(ArmConstants.kPIDLoopIdx));

  }

  private double nextValue = 500;
  private boolean recordTable = false;

  public void toggleRecord(boolean value)
  {
    if (value != recordTable)
    {
      recordTable = value;
      if (recordTable)
      {
        nextValue = 500;
        FFTable.clear();
      }
      else
      {
        System.out.println("Recorded FFTable:");
        System.out.print(FFTable);
      }
    }
  }

  public void setArmZero() {
    checkOLRamp(false);
    armMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double sensor = armMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("Arm Traj Pos", armMotor.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Arm Act Pos", sensor);
    SmartDashboard.putNumber("Arm Error", armMotor.getClosedLoopError());

    if (recordTable)
      if (sensor > nextValue)
      {
        FFTable.addSample(sensor, armMotor.getMotorOutputPercent());
        nextValue += 1000;
      }
  }
}
