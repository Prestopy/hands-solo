package frc.robot.subsystem.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.MotorConfiguration;

public class ArmIOSim implements ArmIO {
    Angle arm = Degrees.of(0);

    public ArmIOSim(MotorConfiguration config) {

    }
    
    @Override
    public Angle getAngle() {
        return arm;
    }

    @Override
    public void moveTo(Angle angle) {
        arm = angle;
    }
    
    @Override
    public void stop() {
        // yay
    }
}
