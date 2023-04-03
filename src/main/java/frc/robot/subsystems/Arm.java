// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public boolean isActive = true;
  static Arm Instance = null;
  public WPI_TalonSRX armMotor;
  private int mode = Constants.IntakeOFF_MODE;
  private boolean deployed = false;
  Encoder m_Encoder;
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
    armMotor.configOpenloopRamp(0.1);
    //armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    //armMotor.setSelectedSensorPosition(0);

    m_Encoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
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

  public void setArm(ControlMode control, double level) {
    armMotor.set(control, level);
  }

  public void armInside() {
    armMotor.set(-0.6);
  }

  public void armOutside() {
    armMotor.set(0.6);
  }

  public void midCone() {

  }

  public void midCube() {

  }

  public void highCone() {

  }

  public void highCube() {

  }

  public void setArmZero() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor", m_Encoder.getDistance());
  }
}
