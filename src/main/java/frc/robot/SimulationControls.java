package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotContainer.baseArm;
import static frc.robot.RobotContainer.distalArm;
import static frc.robot.RobotContainer.proximalArm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimulationControls {
    private static CommandGenericHID HID = new CommandGenericHID(0);
    private static Trigger button1 = HID.button(1);
    private static Trigger button2 = HID.button(2);
    private static Trigger button3 = HID.button(3);

    private static Angle j1 = Degrees.of(0.0);
    private static Angle j2 = Degrees.of(0.0);
    private static Angle j3 = Degrees.of(0.0);

    public static void bind() {
        
    }
}