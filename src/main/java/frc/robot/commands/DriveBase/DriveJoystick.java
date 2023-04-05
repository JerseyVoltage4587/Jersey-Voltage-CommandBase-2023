// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DriveBase;

public class DriveJoystick extends CommandBase {
  /** Creates a new DriveJoystick. */
  public DriveJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveBase.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveBase.getInstance().driveMotors(OI.getInstance().j.getRawAxis(1),OI.getInstance().j.getRawAxis(2), false);
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
