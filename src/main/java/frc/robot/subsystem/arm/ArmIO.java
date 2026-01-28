package frc.robot.subsystem.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public interface ArmIO {
    class ArmIOInputs {
        public Angle baseAngle = Degrees.of(0.0);

        public Angle proximalJointAngle = Degrees.of(0.0);
        public Angle distalJointAngle = Degrees.of(0.0);
        public Angle wristJointAngle = Degrees.of(0.0); // pitch

        public Angle clawAngle = Degrees.of(0.0); // roll
        public Angle clawGrabAngle = Degrees.of(0.0); // claw open/close
    }

    void setBaseAngle(Angle angle);
    void setProximalJointAngle(Angle angle);
    void setDistalJointAngle(Angle angle);
    void setWristJointAngle(Angle angle);

    void setClawAngle(Angle angle);
    void setClawGrabAngle(Angle angle);

    void updateInputs(ArmIOInputs inputs);
}
