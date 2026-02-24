package frc.robot.subsystem.arm;

import edu.wpi.first.units.measure.Angle;

public interface ArmIO {    
    Angle getAngle();
    void moveTo(Angle setpoint);
    void stop();
}
