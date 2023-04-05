// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmToPosition extends CommandBase {
  /** Creates a new MidCone. */

  private double m_position;
  public ArmToPosition(double position) {
    addRequirements(Arm.getInstance());
    m_position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {   
    Arm.getInstance().gotoPosition(m_position, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Arm.getInstance().armMotor.feed();
      SmartDashboard.putNumber("Arm Error", Arm.getInstance().armMotor.getClosedLoopError(ArmConstants.kPIDLoopIdx));
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Arm.getInstance().armMotor.getActiveTrajectoryPosition() - m_position) < 1;
  }
}
