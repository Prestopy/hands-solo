package frc.lib;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyTalon implements LazyCTRE<TalonFX, FeedbackSensorSourceValue> {
    private TalonFX motor, follower;
    private int motorID;

    private TalonFXConfiguration motorConfiguration, followConfiguration;

    private CANcoder canCoder;
    private CANcoderConfiguration canCoderConfiguration;

    public LazyTalon(int motorID, InvertedValue invertedValue) {
        this(motorID, 1.0, invertedValue, 30.0, 30.0);
    }
    /**
     * Constructs a LazyTalon instance with the specified motor identifier and
     * configuration parameters.
     *
     * @param motorID                the device id of the motor controller
     * @param sensorToMechanismRatio the gear ratio of the sensor to the mechanism
     *                               (tell the mech team (driven / driving) *
     *                               planetary product)
     * @param invertedValue          what direction that is positive for the motor
     * @param statorCurrentLimit     the maximum stator current
     * @param supplyCurrentLimit     the maximum supply current
     */
    public LazyTalon(int motorID, double rotorToSensorRatio, InvertedValue invertedValue, double statorCurrentLimit,
            double supplyCurrentLimit) {
        this(motorID, "", rotorToSensorRatio, invertedValue, statorCurrentLimit, supplyCurrentLimit);
    }

    public LazyTalon(int motorID, String canBus, double rotorToSensorRatio, InvertedValue invertedValue,
            double statorCurrentLimit,
            double supplyCurrentLimit) {
        this.motorID = motorID;

        motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Feedback.RotorToSensorRatio = rotorToSensorRatio;

        motorConfiguration.MotorOutput.Inverted = invertedValue;

        motorConfiguration.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        motor = new TalonFX(motorID, canBus);
    }

    @Override
    public LazyTalon withFollower(int followerID, MotorAlignmentValue isInverted) {
        followConfiguration = new TalonFXConfiguration();

        follower = new TalonFX(followerID);
        follower.getConfigurator().apply(followConfiguration);
        follower.setControl(new Follower(motorID, isInverted));

        return this;
    }

    @Override
    public LazyTalon withCANCoder(int CANCoderID, FeedbackSensorSourceValue sensorType, double magnetOffset,
            SensorDirectionValue sensorDirection) {
        canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = magnetOffset;
        canCoderConfiguration.MagnetSensor.SensorDirection = sensorDirection;

        canCoder = new CANcoder(CANCoderID);
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.Feedback.FeedbackRemoteSensorID = CANCoderID;
        motorConfiguration.Feedback.FeedbackSensorSource = sensorType;

        return this;
    }

    @Override
    public LazyTalon withCANCoder(int CANCoderID, FeedbackSensorSourceValue sensorType, double magnetOffset,
            SensorDirectionValue sensorDirection, double discontinuityPoint, double rotorToSensorRatio) {
        withCANCoder(CANCoderID, sensorType, magnetOffset, sensorDirection);

        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.Feedback.RotorToSensorRatio = rotorToSensorRatio;

        return this;
    }

    // Motion magic voltage
    @Override
    public LazyTalon withMotionMagicConfiguration(double p, double i, double d, double s, double g, double v, double a,
            GravityTypeValue gravityType, double cruiseVelocity, double maxAcceleration) {
        motorConfiguration.Slot0.kP = p;
        motorConfiguration.Slot0.kI = i;
        motorConfiguration.Slot0.kD = d;
        motorConfiguration.Slot0.kS = s;
        motorConfiguration.Slot0.kG = g;
        motorConfiguration.Slot0.kV = v;
        motorConfiguration.Slot0.kA = a;
        motorConfiguration.Slot0.GravityType = gravityType;

        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        return this;
    }

    @Override
    public LazyTalon withMotionMagicConfiguration(double p, double i, double d, double s, double g, double v, double a,
            double expoV, double expoA, GravityTypeValue gravityType, double cruiseVelocity, double maxAcceleration) {
        motorConfiguration.Slot0.kP = p;
        motorConfiguration.Slot0.kI = i;
        motorConfiguration.Slot0.kD = d;
        motorConfiguration.Slot0.kS = s;
        motorConfiguration.Slot0.kG = g;
        motorConfiguration.Slot0.kV = v;
        motorConfiguration.Slot0.kA = a;
        motorConfiguration.Slot0.GravityType = gravityType;

        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        motorConfiguration.MotionMagic.MotionMagicExpo_kV = expoV;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = expoA;

        return this;
    }

    @Override
    public LazyTalon withMotionMagicConfiguration(double p, double i, double d, double s, double g, double v, double a, double expoV, double expoA, GravityTypeValue gravityType, double cruiseVelocity, double maxAcceleration, int slot) {
        switch (slot) {
            case 0: 
            motorConfiguration.Slot0.kP = p;
            motorConfiguration.Slot0.kI = i;
            motorConfiguration.Slot0.kD = d;
            motorConfiguration.Slot0.kS = s;
            motorConfiguration.Slot0.kG = g;
            motorConfiguration.Slot0.kV = v;
            motorConfiguration.Slot0.kA = a;
            motorConfiguration.Slot0.GravityType = gravityType;
            case 1: 
            motorConfiguration.Slot1.kP = p;
            motorConfiguration.Slot1.kI = i;
            motorConfiguration.Slot1.kD = d;
            motorConfiguration.Slot1.kS = s;
            motorConfiguration.Slot1.kG = g;
            motorConfiguration.Slot1.kV = v;
            motorConfiguration.Slot1.kA = a;
            motorConfiguration.Slot1.GravityType = gravityType;
            case 2: 
            motorConfiguration.Slot2.kP = p;
            motorConfiguration.Slot2.kI = i;
            motorConfiguration.Slot2.kD = d;
            motorConfiguration.Slot2.kS = s;
            motorConfiguration.Slot2.kG = g;
            motorConfiguration.Slot2.kV = v;
            motorConfiguration.Slot2.kA = a;
            motorConfiguration.Slot2.GravityType = gravityType;
        }
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        motorConfiguration.MotionMagic.MotionMagicExpo_kV = expoV;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = expoA;

        return this;
    }

    @Override
    public LazyTalon withLimitSwitch(boolean forwardLimitEnable, boolean forwardLimitAutosetPositionEnable,
            double forwardLimitAutosetPositionValue, boolean reverseLimitEnable,
            boolean reverseLimitAutosetPositionEnable, double reverseLimitAutosetPositionValue) {
        motorConfiguration.HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnable;
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forwardLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardLimitAutosetPositionValue;

        motorConfiguration.HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnable;
        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverseLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseLimitAutosetPositionValue;

        return this;
    }

    @Override
    public LazyTalon withSoftLimits(boolean forwardSoftLimitEnable, double forwardSoftLimit,
            boolean reverseSoftLimitEnable, double reverseSoftLimit) {
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit;

        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit;

        return this;
    }

    @Override
    public LazyTalon withCustomConfiguration(ParentConfiguration configuration) {
        motorConfiguration = (TalonFXConfiguration) configuration;

        return this;
    }

    @Override
    public TalonFXConfiguration getConfiguration() {
        return motorConfiguration;
    }

    @Override
    public LazyTalon build() {
        motor.getConfigurator().apply(motorConfiguration);
        motor.setNeutralMode(NeutralModeValue.Brake);
        if (motor.getIsProLicensed().getValue() ==  false) DriverStation.reportWarning("Motor" + motor.getDeviceID() + " on CANbus" + motor.getNetwork(), false);
        BaseStatusSignal.setUpdateFrequencyForAll(250, motor.getPosition(), motor.getVelocity(),
                motor.getAcceleration(), motor.getStatorCurrent(), motor.getSupplyCurrent());
        motor.optimizeBusUtilization();
        for (int i = 0; i < 5; i++) {
            var error = motor.getConfigurator().apply(motorConfiguration);
            if (error.isOK()) {
                break;
            } else {
                DriverStation.reportWarning(
                        "Warning: Your motor configuration on motor " + motor.getDeviceID() + " was not applied!",
                        false);
            }
        }
        return this;
    }

    @Override
    public void moveTo(Angle setpoint) {
        motor.setControl(new MotionMagicExpoVoltage(setpoint).withEnableFOC(true));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(new MotionMagicVelocityVoltage(velocity).withEnableFOC(true));
    }

    public void moveWithTorqueCurrentFOC(Angle setpoint) {
        motor.setControl(new MotionMagicTorqueCurrentFOC(setpoint));
    }

    public void moveWithTorqueCurrentVelocityFOC(AngularVelocity velocity) {
        motor.setControl(new MotionMagicVelocityTorqueCurrentFOC(velocity));
    }

    public void moveWithRawPIDF(AngularVelocity velocity) {
        motor.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void setPercent(double speed) {
        motor.set(speed);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void idle() {
        motor.setControl(new VoltageOut(Volts.of(this.motorConfiguration.Slot0.kS)));
    }

    @Override
    public void setBrake() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setCoast() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public Angle getFeedbackPosition() {
        return motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getFeedbackVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public AngularAcceleration getFeedbackAcceleration() {
        return motor.getAcceleration().getValue();
    }

    @Override
    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public CANcoder getCanCoder() {
        return canCoder;
    }

    @Override
    public TalonFX getFollower() {
        return follower;
    }

    @Override
    public void updateTelemetry(StatusSignalMotorTelemetry telemetry) {
        telemetry.position = motor.getPosition();
        telemetry.velocity = motor.getVelocity();
        telemetry.acceleration = motor.getAcceleration();
        telemetry.statorCurrent = motor.getStatorCurrent();
        telemetry.supplyCurrent = motor.getSupplyCurrent();
    }
    @Override
    public void updateTelemetry(MotorTelemetry telemetry) {
        telemetry.position = motor.getPosition().getValue();
        telemetry.velocity = motor.getVelocity().getValue();
        telemetry.acceleration = motor.getAcceleration().getValue();
        telemetry.statorCurrent = motor.getStatorCurrent().getValue();
        telemetry.supplyCurrent = motor.getSupplyCurrent().getValue();
    }
}