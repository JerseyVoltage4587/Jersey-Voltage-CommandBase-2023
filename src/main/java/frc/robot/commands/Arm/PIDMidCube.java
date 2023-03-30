// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Arm;

public class PIDMidCube extends CommandBase {
  Arm m_arm;
  OI m_oi;

  /** Creates a new PIDMidCube. */
  public PIDMidCube() {
    m_arm = Arm.getInstance();
    m_oi = OI.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_oi.k.getRawButton(1)) {
      m_arm.armMotor.set(ControlMode.MotionMagic, 25000);
      
    } else {
      m_arm.armMotor.set(ControlMode.PercentOutput, m_oi.k.getRawAxis(1));
    }
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
