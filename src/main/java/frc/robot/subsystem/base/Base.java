package frc.robot.subsystem.base;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
 private BaseIO io;

    public Base(BaseIO io) {
        this.io = io;
    }

    public Command setAngle(Angle angle) {
        return runEnd(
            () -> io.moveTo(angle),
            () -> io.stop()
        );
    }

    public Command stop() {
        return runOnce(() -> io.stop());
    }
}
