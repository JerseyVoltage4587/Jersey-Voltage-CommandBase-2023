// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveBase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutoBalance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAutoBalance extends SequentialCommandGroup {
  /** Creates a new DriveAutoBalance. */
  public DriveAutoBalance(boolean reversed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double dir = reversed ? -1 : 1;
    addCommands(
      new DriveCommand(dir * AutoBalance.robotSpeedFast, 0, false).until(AutoBalance.getInstance().climbingTrigger),
      new DriveCommand(dir * AutoBalance.robotSpeedSlow, 0, false).until(AutoBalance.getInstance().levelingStartTrigger),
      new DriveCommand(-dir * AutoBalance.robotSpeedSlow, 0, false).withTimeout(0.2),
      new FinishBalance()
    );
  }
}
