// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class MoveDistance extends CommandBase {
  DriveBase m_DriveBase;
  double distance;
  /** Creates a new MoveDistance. */
  public MoveDistance(double x) {
    distance = x;
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
    m_DriveBase.MoveDistance(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
