// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmIOReal;
import frc.robot.subsystem.arm.ArmIOSim;
import frc.robot.subsystem.base.Base;
import frc.robot.subsystem.claw.Claw;

public class RobotContainer {
  public static Base base;
  public static Arm baseArm;
  public static Arm proximalArm;
  public static Arm distalArm;
  public static Claw claw;

  public RobotContainer() {
    if (RobotBase.isReal()) {
      baseArm = new Arm(new ArmIOReal(ArmConstants.Configurations.BASE_ARM));
    } else {
      baseArm = new Arm(new ArmIOSim(ArmConstants.Configurations.BASE_ARM));
    }

    configureBindings();
  }

  private void configureBindings() {
    if (RobotBase.isReal()) {
      
    } else {
      SimulationControls.bind();
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
