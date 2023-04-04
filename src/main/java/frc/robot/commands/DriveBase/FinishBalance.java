// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoBalance;
import frc.robot.subsystems.DriveBase;

public class FinishBalance extends CommandBase {
  /** Creates a new FinishLevel. */
  public FinishBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveBase.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tilt = AutoBalance.getInstance().getTilt();
    if (tilt < AutoBalance.levelDegree / 2)
      DriveBase.getInstance().driveMotors(0, 0, true);
    else
      DriveBase.getInstance().driveMotors(AutoBalance.robotSpeedNudge * Math.signum(tilt), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return AutoBalance.getInstance().levelingEndTrigger.getAsBoolean();
  }
}
