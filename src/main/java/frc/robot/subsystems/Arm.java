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
import frc.robot.OI;

public class Arm extends SubsystemBase {
  public boolean isActive = true;
  static Arm Instance = null;
  public WPI_TalonSRX armMotor;
  private int mode = Constants.IntakeOFF_MODE;
  private boolean deployed = false;
  Encoder m_Encoder;
  private OI m_controller;
  /** Creates a new Arm. */
  
  public Arm() {
    if (isActive == false) {
      return;
    }
    m_controller = OI.getInstance();
    armMotor = new WPI_TalonSRX(8);
    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    armMotor.setSensorPhase(true);
    // armMotor.configMotionCruiseVelocity(Constants.kCruiseVelocity, 30);
    // armMotor.configMotionAcceleration(Constants.kCruiseAcceleration, 30);
    // armMotor.set(ControlMode.MotionMagic, 25000);
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

  public void armInside() {
    armMotor.set(-0.4);
  }

  public void armOutside() {
    armMotor.set(0.4);
  }

  public void midCone() {
    // if (m_controller.k.getRawButton(2)) {
    //   double targetPos = OI.getInstance().k.getRawAxis(1) * 4096 * 10.0;
    //   armMotor.set(ControlMode.MotionMagic, targetPos);

    //   System.out.println("*****");
    //   System.out.println(targetPos);
    //   System.out.println(armMotor.getClosedLoopError(0));

    // } else {
    //   armMotor.set(ControlMode.PercentOutput, OI.getInstance().k.getRawAxis(1));
    // }
  }

  public void midCube() {
    // armMotor.set(ControlMode.MotionMagic, );
  }

  public void highCone() {
    //armMotor.set(ControlMode.MotionMagic, );
  }

  public void highCube() {
    //armMotor.set(ControlMode.MotionMagic, );
  }

  public void setArmPositionZero() {
    //armMotor.set(ControlMode.MotionMagic, 0);
  }

  public void setArmSpeedZero() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor", m_Encoder.getDistance());
  }
}
