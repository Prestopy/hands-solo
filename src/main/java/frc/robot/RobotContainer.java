// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmIO.ArmIOInputs;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Color;

public class RobotContainer {
  public static Arm ARM;

  private final Mechanism2d mech = new Mechanism2d(20, 10);
  private final MechanismLigament2d arm1Ligament =
    mech.getRoot("base", 5, 2.5).append(
        new MechanismLigament2d("arm1", 3, 0, 6, new Color8Bit(Color.kPurple))
    );
  private final MechanismLigament2d arm2Ligament =
    arm1Ligament.append(
        new MechanismLigament2d("arm2", 3, 0, 6, new Color8Bit(Color.kOrange))
    );
  private final MechanismLigament2d claw1Ligament = 
    arm2Ligament.append(
      new MechanismLigament2d("claw1", 1, 0, 6, new Color8Bit(Color.kGreen))
    );
  private final MechanismLigament2d claw2Ligament = 
    arm2Ligament.append(
      new MechanismLigament2d("claw2", 1, 0, 6, new Color8Bit(Color.kGreen))
    );


  public RobotContainer() {
    if (RobotBase.isReal()) {
      ARM = new Arm(new frc.robot.subsystem.arm.ArmIOReal());
    } else {
      ARM = new Arm(new frc.robot.subsystem.arm.ArmIOSim());
    }

    SmartDashboard.putData("Mech2d", mech);

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

  // int counter = 0;
  public void updateMech2d() {
    ArmIOInputs inputs = ARM.getInputs();
    arm1Ligament.setAngle(inputs.proximalJointAngle.in(Degrees));
    arm2Ligament.setAngle(inputs.distalJointAngle.in(Degrees));

    claw1Ligament.setAngle(90.0+inputs.wristJointAngle.in(Degrees));
    claw2Ligament.setAngle(270.0+inputs.wristJointAngle.in(Degrees));
    // arm1Ligament.setAngle(counter);
    // arm2Ligament.setAngle(counter);
    // counter = (counter + 1) % 360;
  }
}
