package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Current;

public record MotorConfiguration(
    int MOTOR_ID,

    Current STATOR_CURRENT_LIMIT,
    Current SUPPLY_CURRENT_LIMIT,

    double GEAR_RATIO,

    InvertedValue INVERTED_VALUE, 
    double SENSOR_TO_MECHANISM_RATIO,
    
    double kS,
    double kG, 
    double kV,
    double kA,
    double kP,
    double kI,
    double kD,

    double EXPO_VELOCITY,
    double EXPO_ACCELERATION,

    GravityTypeValue GRAVITY_TYPE,
    
    double CRUISE_VELOCITY,
    double MAX_ACCELERATION
) {}
