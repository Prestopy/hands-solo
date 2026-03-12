package frc.robot.subsystem.claw;

import edu.wpi.first.units.measure.Angle;

public interface ClawIO {    
    Angle getAngle();
    void moveTo(Angle setpoint);
    void stop();
}
    

