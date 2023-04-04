// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */

  private double m_forward, m_turn;
  private boolean m_percentMode;

  public DriveCommand(double forward, double turn, boolean percentMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_forward = forward;
    m_turn = turn;
    m_percentMode = percentMode;
    addRequirements(DriveBase.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveBase.getInstance().toggleMode(m_percentMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveBase.getInstance().driveMotors(m_forward, m_turn, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveBase.getInstance().driveMotors(m_forward, m_turn, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
