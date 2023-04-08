// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
    armMotor.enableCurrentLimit(false);
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
		armMotor.configMotionCruiseVelocity(ArmConstants.kCruiseVelocity, ArmConstants.kTimeoutMs);
		armMotor.configMotionAcceleration(ArmConstants.kCruiseAccel, ArmConstants.kTimeoutMs);
    armMotor.configMotionSCurveStrength(ArmConstants.kSmooth);

    int closedLoopTimeMs = 2;
		armMotor.configClosedLoopPeriod(0, closedLoopTimeMs, ArmConstants.kTimeoutMs);

    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    armMotor.configClearPositionOnLimitR(true, ArmConstants.kTimeoutMs);
		/* Zero the sensor once on robot boot up */
		armMotor.setSelectedSensorPosition(0, ArmConstants.kPIDLoopIdx, ArmConstants.kTimeoutMs);
    FFTable.addSample(0.0,0.0);
    FFTable.addSample(10058.0, 0.11827956989247312);
    FFTable.addSample(20196.0, 0.1348973607038123);
    FFTable.addSample(30035.0, 0.16911045943304007);
    FFTable.addSample(40036.0, 0.2072336265884653);
    FFTable.addSample(50074.0, 0.23069403714565004);
    FFTable.addSample(60304.0, 0.24633431085043989);
    FFTable.addSample(70278.0, 0.26295210166177907);
    FFTable.addSample(80286.0, 0.2727272727272727);
    FFTable.addSample(90212.0, 0.27956989247311825);
    FFTable.addSample(100069.0, 0.2873900293255132);
    FFTable.addSample(110013.0, 0.29423264907135877);
    FFTable.addSample(120431.0, 0.3040078201368524);
    FFTable.addSample(130323.0, 0.31573802541544477);
    FFTable.addSample(140043.0, 0.33235581622678395);
    FFTable.addSample(150125.0, 0.44086021505376344);
    FFTable.addSample(160011.0, 0.4506353861192571);
    FFTable.addSample(170191.0, 0.4613880742913001);
    FFTable.addSample(180125.0, 0.544477028347996);
    FFTable.addSample(190438.0, 0.5532746823069403);
     }
  
  public void setArm(ControlMode control, double level) {
    armMotor.set(control, level);
  }

  boolean homed = false;

  public boolean getHomed()
  {
    return homed;
  }
  
  public void setHomed(boolean val)
  {
    homed = val;
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
    double sensor = armMotor.getSelectedSensorPosition();
    armMotor.set(ControlMode.PercentOutput, sensor < 40000 ? ArmConstants.kRetract / 1.25 : ArmConstants.kRetract);
  }

  public void armOutside() {
    checkOLRamp(false);
    armMotor.set(ControlMode.PercentOutput, ArmConstants.kExtend);
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

    armMotor.set(ControlMode.MotionMagic, Math.max(0, Math.min(position - adj, 200000))    , DemandType.ArbitraryFeedForward, FFTable.getSample(armMotor.getSelectedSensorPosition()).orElse(0.0) * 0.75);
    SmartDashboard.putNumber("Arm Error", armMotor.getClosedLoopError(ArmConstants.kPIDLoopIdx));

  }

  private double nextValue = 10000;
  private boolean recordTable = false;

  public void toggleRecord(boolean value)
  {
    if (value != recordTable)
    {
      recordTable = value;
      if (recordTable)
      {
        System.out.println("FFTable Cleared");
        nextValue = 10000;
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
    double motOut = armMotor.getMotorOutputPercent();
    SmartDashboard.putNumber("Arm Traj Pos", armMotor.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Arm Act Pos", sensor);
    SmartDashboard.putNumber("Arm Error", armMotor.getClosedLoopError());
    SmartDashboard.putNumber("Arm Output", motOut);
    SmartDashboard.putNumber("Arm OutV", armMotor.getMotorOutputVoltage());

    if (atReverseLimit())
    {
      homed = true;
    }

    if (recordTable)
      if (sensor > nextValue)
      {
        System.out.println("Recorded: " + sensor + ", " + motOut);
        FFTable.addSample(sensor, armMotor.getMotorOutputPercent());
        nextValue += 10000;
      }
  }
}
