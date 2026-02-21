package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ClimbConstants;

public class ClimbIOSparkFlex implements ClimbIO {
    private SparkFlex motor;

    public ClimbIOSparkFlex() {
        motor = new SparkFlex(ClimbConstants.MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPosition(double position) {
        motor.getEncoder().setPosition(position);
    }

    public void updateInputs(ClimbIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.RPM = motor.getEncoder().getVelocity();
        inputs.absolutePosition = motor.getEncoder().getPosition();
    }

    public void runVoltage(double volts) {
        motor.setVoltage(volts);
    }
}