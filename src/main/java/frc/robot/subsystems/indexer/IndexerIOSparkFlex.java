package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IndexerConstants;

public class IndexerIOSparkFlex implements IndexerIO {
    private final SparkFlex motor;

    public IndexerIOSparkFlex() {
        this.motor = new SparkFlex(IndexerConstants.MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(IndexerIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.RPM = motor.getEncoder().getVelocity();
    }

    public void runVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
