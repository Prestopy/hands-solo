package frc.robot.subsystem.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.MotorConfiguration;

public class ArmConstants {
    public static final Angle SETPOINT_TOLERANCE = Degrees.of(2.0);
    public class Configurations {
        public static final MotorConfiguration BASE_ARM = new MotorConfiguration(
            16,

            Amps.of(30), // Stator limit
            Amps.of(30), // Supply limit

            1.0,

            InvertedValue.CounterClockwise_Positive, // Inverted value
            1.0,
            
            0, // S
            0, // G
            0, // V
            0, // A
            1, // P
            0, // I
            0, // D

            0.12,
            0.01,

            GravityTypeValue.Elevator_Static, // Gravity type
            
            10,
            20
        );
        public static final MotorConfiguration PROXIMAL_ARM = new MotorConfiguration(
            16,

            Amps.of(30), // Stator limit
            Amps.of(30), // Supply limit

            1.0,

            InvertedValue.CounterClockwise_Positive, // Inverted value
            1.0,
            
            0, // S
            0, // G
            0, // V
            0, // A
            1, // P
            0, // I
            0, // D

            0.12,
            0.01,

            GravityTypeValue.Elevator_Static, // Gravity type
            
            10,
            20
        );
        public static final MotorConfiguration DISTAL_ARM = new MotorConfiguration(
            16,

            Amps.of(30), // Stator limit
            Amps.of(30), // Supply limit

            1.0,

            InvertedValue.CounterClockwise_Positive, // Inverted value
            1.0,
            
            0, // S
            0, // G
            0, // V
            0, // A
            1, // P
            0, // I
            0, // D

            0.12,
            0.01,

            GravityTypeValue.Elevator_Static, // Gravity type
            
            10,
            20
        );
    }
}
