// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class MoveDistance extends CommandBase {
  DriveBase m_DriveBase;
  double distance, velocity;
  /** Creates a new MoveDistance. */
  public MoveDistance(double x, double speed) {
    distance = x;
    velocity = speed;
    m_DriveBase = DriveBase.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveBase.zeroDriveSensors(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveBase.MoveDistance(distance, velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveBase.setLeftMotorLevel(0);
    m_DriveBase.setRightMotorLevel(0);
    m_DriveBase.zeroDriveSensors(false);
    SmartDashboard.putNumber("Final Distance", m_DriveBase.getAverageDistanceInches());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_DriveBase.getAverageDistanceInches()) > Math.abs(distance)) {
      return true;
    } else {
    return false;
    }
  }
}
