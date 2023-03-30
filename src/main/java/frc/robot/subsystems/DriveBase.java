/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
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
  private long m_lastLogTime = 0;
  private String layout = "";


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
    //driveLeftTalon.restoreFactoryDefaults();
    //driveRightTalon.restoreFactoryDefaults();
    //driveLeftVictor.follow(driveLeftTalon);
    //driveRightVictor.follow(driveRightTalon);
    driveLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    driveRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    driveLeftTalon.setSelectedSensorPosition(0);
    driveRightTalon.setSelectedSensorPosition(0);
    driveLeftTalon.set(0);
    driveRightTalon.set(0);
    driveLeftTalon.setNeutralMode(NeutralMode.Brake);
    driveRightTalon.setNeutralMode(NeutralMode.Brake);
    driveLeftTalon.setInverted(false);
    driveLeftVictor.setInverted(false);
    driveRightTalon.setInverted(false);
    driveRightVictor.setInverted(false);
    driveLeftTalon.enableCurrentLimit(true);
    driveLeftTalon.configContinuousCurrentLimit(30);
    driveLeftVictor.follow(driveLeftTalon);
    driveRightTalon.enableCurrentLimit(true);
    driveRightTalon.configContinuousCurrentLimit(30);
    driveRightVictor.follow(driveRightTalon);
    driveLeftTalon.configOpenloopRamp(0.1);
    driveRightTalon.configOpenloopRamp(0.1);
    m_drive = new DifferentialDrive(driveLeftTalon, driveRightTalon);
    m_drive.setSafetyEnabled(false);
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
      return driveLeftTalon.getSelectedSensorPosition() / 10.75;
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
      return driveRightTalon.getSelectedSensorPosition() / 10.75;
    }

    public double getRightDistanceInches() {
      return getRightEncoder() * Constants.DriveBaseWheelDiameter * Math.PI;
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
    driveLeftTalon.setSelectedSensorPosition(0);
    driveRightTalon.setSelectedSensorPosition(0);
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

  public void MoveDistance(double x) {
    
    double leftInches = Math.abs(getLeftDistanceInches());
    double rightInches = Math.abs(getRightDistanceInches());

    double averageInches = (leftInches + rightInches) / 2;
    if (averageInches > x) 
    {
      driveLeftTalon.set(0);
      driveRightTalon.set(0);
    } else {
      driveLeftTalon.set(-0.3);
      driveRightTalon.set(0.3);
    }
  }

  public void driveMotors(double forward, double turn) {
    double left = forward - turn;
    double right = forward + turn;

    driveLeftTalon.set(-left);
    driveLeftVictor.follow(driveLeftTalon);
    driveRightTalon.set(right);
    driveRightVictor.follow(driveRightTalon);
  }
}
