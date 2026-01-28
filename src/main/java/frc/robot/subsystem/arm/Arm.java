package frc.robot.subsystem.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.arm.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputs inputs;

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputs();
    }

    public Command setBaseAngle(double angle) {
        return runOnce(() -> io.setBaseAngle(Degrees.of(angle)));
    }

    public Command setProximalJointAngle(double angle) {
        return runOnce(() -> io.setProximalJointAngle(Degrees.of(angle)));
    }

    public Command setDistalJointAngle(double angle) {
        return runOnce(() -> io.setDistalJointAngle(Degrees.of(angle)));
    }

    public Command setWristJointAngle(double angle) {
        return runOnce(() -> io.setWristJointAngle(Degrees.of(angle)));
    }

    public Command setClawAngle(double angle) {
        return runOnce(() -> io.setClawAngle(Degrees.of(angle)));
    }

    public Command setClawGrabAngle(double angle) {
        return runOnce(() -> io.setClawGrabAngle(Degrees.of(angle)));
    }


    public ArmIOInputs getInputs() {
        return inputs;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
