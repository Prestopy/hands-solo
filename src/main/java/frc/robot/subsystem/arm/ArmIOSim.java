package frc.robot.subsystem.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class ArmIOSim implements ArmIO {
    Angle baseAngle = Degrees.of(0);
    Angle proximalJointAngle = Degrees.of(0);
    Angle distalJointAngle = Degrees.of(0);
    Angle wristJointAngle = Degrees.of(0);
    Angle clawAngle = Degrees.of(0);
    Angle clawGrabAngle = Degrees.of(0);
    
    // BASE ROTATIUON
    @Override
    public void setBaseAngle(Angle angle) {
        baseAngle = angle;
    }

    // arm joint 1
    @Override
    public void setProximalJointAngle(Angle angle) {
        proximalJointAngle = angle;
    }

    // arm joint 2
    @Override
    public void setDistalJointAngle(Angle angle) {
        distalJointAngle = angle;
    }

    // arm joint 3
    @Override
    public void setWristJointAngle(Angle angle) {
        wristJointAngle = angle;
    }

    // WRIST ROTATION
    @Override
    public void setClawAngle(Angle angle) {
        clawAngle = angle;
    }

    @Override
    public void setClawGrabAngle(Angle angle) {
        clawGrabAngle = angle;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.baseAngle = baseAngle;
        inputs.proximalJointAngle = proximalJointAngle;
        inputs.distalJointAngle = distalJointAngle;
        inputs.wristJointAngle = wristJointAngle;
        inputs.clawAngle = clawAngle;
        inputs.clawGrabAngle = clawGrabAngle;
    }
}
