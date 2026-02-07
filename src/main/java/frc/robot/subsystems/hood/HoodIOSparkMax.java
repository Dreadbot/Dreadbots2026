package frc.robot.subsystems.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.FlywheelConstants;

public class HoodIOSparkMax implements HoodIO {
    private final SparkMax motor;

    public HoodIOSparkMax() {
        motor = new SparkMax(FlywheelConstants.MOTOR_ID_1, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.angle = motor.getEncoder().getPosition() * 360.0; // Convert total rotations to degrees
        inputs.velocity = motor.getEncoder().getVelocity();
    }
}
