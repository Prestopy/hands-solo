package frc.robot.subsystem.base;

import edu.wpi.first.units.measure.Angle;

public interface BaseIO {    
    Angle getAngle();
    void moveTo(Angle setpoint);
    void stop();
}
    

