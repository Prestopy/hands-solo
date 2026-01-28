package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.RobotContainer.ARM;

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

    private static int j1 = 0;
    private static int j2 = 0;
    private static int j3 = 0;

    public static void bind() {
        button1.whileTrue(
            new InstantCommand(() -> {
                ARM.setProximalJointAngle(j1).schedule();
                j1 += 5;
            }).repeatedly()
        );

        button2.whileTrue(
            new InstantCommand(() -> {
                ARM.setDistalJointAngle(j2).schedule();
                j2 += 5;
            }).repeatedly()
        );

        button3.whileTrue(
            new InstantCommand(() -> {
                ARM.setWristJointAngle(j3).schedule();
                j3 += 5;
            }).repeatedly()
        );
    }
}