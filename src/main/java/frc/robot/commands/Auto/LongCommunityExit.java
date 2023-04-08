// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.InstantArmInside;
import frc.robot.commands.Arm.InstantArmOutside;
import frc.robot.commands.Arm.InstantSetArmZero;
import frc.robot.commands.DriveBase.MoveDistance;
import frc.robot.commands.Intake.InstantCubeOut;
import frc.robot.commands.Intake.InstantSetIntakeZero;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LongCommunityExit extends SequentialCommandGroup {
  /** Creates a new LongCommunityExit. */
  public LongCommunityExit() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantArmOutside(),
    new WaitCommand(2),
    new InstantCubeOut(),
    new WaitCommand(1),
    new InstantArmInside(),
    new WaitCommand(0.5),
    new InstantSetIntakeZero(),
    new InstantSetArmZero(),
    new WaitCommand(2),
    new MoveDistance(250, 0.3));
  }
}
