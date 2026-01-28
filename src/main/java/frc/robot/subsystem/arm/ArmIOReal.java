package frc.robot.subsystem.arm;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

public class ArmIOReal implements ArmIO {
    private TalonFX baseMotor;

    private TalonFX proximalJointMotor;
    private TalonFX distalJointMotor;
    private TalonFX wristJointMotor;

    private TalonFX clawRotationMotor;
    private TalonFX clawGrabMotor;

    @Override
    public void setBaseAngle(Angle angle) {
        // Implement hardware control to set the base angle of the arm
    }

    @Override
    public void setProximalJointAngle(Angle angle) {
        // Implement hardware control to set the proximal angle of the arm
    }

    @Override
    public void setDistalJointAngle(Angle angle) {
        // Implement hardware control to set the distal angle of the arm
    }
    
    @Override
    public void setWristJointAngle(Angle angle) {
        // Implement hardware control to set the wrist angle of the arm
    }

    @Override
    public void setClawAngle(Angle angle) {
        // Implement hardware control to set the wrist base angle of the arm
    }

    @Override
    public void setClawGrabAngle(Angle angle) {
        // Implement hardware control to set the claw grab angle of the arm
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.baseAngle = baseMotor.getPosition().getValue();
        inputs.distalJointAngle = distalJointMotor.getPosition().getValue();
        inputs.proximalJointAngle = proximalJointMotor.getPosition().getValue();
        inputs.wristJointAngle = wristJointMotor.getPosition().getValue();
        inputs.clawAngle = clawRotationMotor.getPosition().getValue();
        inputs.clawGrabAngle = clawGrabMotor.getPosition().getValue();
    }
}
