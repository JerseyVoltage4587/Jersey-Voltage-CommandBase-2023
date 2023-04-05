// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ArmInside;
import frc.robot.commands.Arm.ArmOutside;
import frc.robot.commands.Arm.ArmToPosition;
import frc.robot.commands.Arm.JoyDrive;
import frc.robot.commands.Arm.SetArmZero;
import frc.robot.commands.Intake.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;

public class OI extends CommandBase {
  /** Creates a new OI. */
  DriveBase m_drive;
  static OI Instance = null;
  public Joystick j, k;
  public Trigger jButtonY, jButtonB, jButtonA, jButtonX, jLeftBumper, jRightBumper, jLeftTrigger, jRightTrigger,
  jMinusButton, jPlusButton, jLeftStickButton, jRightStickButton;

  public Trigger kButtonY, kButtonB, kButtonA, kButtonX, kLeftBumper, kRightBumper, kLeftTrigger, kRightTrigger,
  kMinusButton, kPlusButton, kLeftStickButton, kRightStickButton;

  public OI() {
    // Use addRequirements() here to declare subsystem dependencies.
    j = new Joystick(0);
    k = new Joystick(1);

    jButtonY = new JoystickButton(j, 1);
    jButtonB = new JoystickButton(j, 2);
    jButtonA = new JoystickButton(j, 3);
    jButtonX = new JoystickButton(j, 4);
    jLeftBumper = new JoystickButton(j, 5);
    jRightBumper = new JoystickButton(j, 6);
    jLeftTrigger = new JoystickButton(j, 7);
    jRightTrigger = new JoystickButton(j, 8);
    jMinusButton = new JoystickButton(j, 9);
    jPlusButton = new JoystickButton(j, 10);
    jLeftStickButton = new JoystickButton(j, 11);
    jRightStickButton = new JoystickButton(j, 12);

    kButtonY = new JoystickButton(k, 1);
    kButtonB = new JoystickButton(k, 2);
    kButtonA = new JoystickButton(k, 3);
    kButtonX = new JoystickButton(k, 4);
    kLeftBumper = new JoystickButton(k, 5);
    kRightBumper = new JoystickButton(k, 6);
    kLeftTrigger = new JoystickButton(k, 7);
    kRightTrigger = new JoystickButton(k, 8);
    kMinusButton = new JoystickButton(k, 9);
    kPlusButton = new JoystickButton(k, 10);
    kLeftStickButton = new JoystickButton(k, 11);
    kRightStickButton = new JoystickButton(k, 12);

    kLeftTrigger.whileTrue(new CubeIn());
    kRightTrigger.whileTrue(new CubeOut());
    kMinusButton.onTrue(new HoldCone());
    kPlusButton.onTrue(new HoldCube());
    kLeftBumper.whileTrue(new ArmOutside());
    kRightBumper.whileTrue(new ArmInside());
    kButtonY.whileTrue(new JoyDrive());
    kButtonA.whileTrue(new ArmToPosition(5000)).onFalse(new SetArmZero());

    kButtonB.toggleOnTrue(new StartEndCommand(() -> Arm.getInstance().toggleRecord(true), () -> Arm.getInstance().toggleRecord(true))); 

  }

  public static OI getInstance() {
    if(Instance == null) {
      synchronized (OI.class) {
          Instance = new OI();
      }
    }
    return Instance;
  }


}
